import os.path
import shutil
import warnings

import click
import logging
import xarray as xr
import pandas as pd
import numpy as np
import datetime as dt
import parse
import importlib.resources

import starpas.utils
import starpas.ftp
import starpas.data


logger = logging.getLogger(__name__)

DEFAULT_CONFIG = fn_config = os.path.join(
        importlib.resources.files("starpas"),
        "share/starpas_config.json"
    )
def _configure(config):
    if config is None:
        config = starpas.utils.get_default_config()
    else:
        config = starpas.utils.read_json(os.path.abspath(config))
        config = starpas.utils.merge_config(config)
    return config

# initialize commandline interface
@click.version_option()
@click.group("starpas")
def cli():
    pass

@cli.command("downloadraw")
@click.argument("start_date", nargs=1)
@click.argument("output_path", nargs=1)
@click.option("--config", "-c", type=click.Path(dir_okay=False, exists=True),
              help="Config file - will merge and override the default config.")
def downloadraw(start_date: str,
                output_path: str,
                config):
    config = _configure(config)
    starpas.utils.init_logger(config)

    fnames = starpas.ftp.fnames_missing(pd.to_datetime(start_date), output_path)
    starpas.ftp.read_from_ftp(
        fnames,
        config=config,
        parse_data=False,
        store_files=output_path,
        check_ftp=True,
    )


@cli.command("raw2l1a")
@click.argument("input_files", nargs=-1)
@click.argument("output_path", nargs=1)
@click.option("--config", "-c", type=click.Path(dir_okay=False, exists=True),
              help="Config file - will merge and override the default config.")
def raw2l1a(input_files,
                output_path: str,
                config):
    config = _configure(config)
    starpas.utils.init_logger(config)

    # scan files
    fdates = []
    for fn in input_files:
        fname_info = parse.parse(
            config["fname_raw"],
            os.path.basename(fn)
        ).named
        fdates.append(pd.to_datetime(np.datetime64(fname_info["dt"]).astype("datetime64[D]")))

    unique_out = np.unique_inverse(fdates)
    udates = unique_out.values
    uidx = unique_out.inverse_indices

    for i, date in enumerate(udates):
        udate_files = input_files[uidx==i]
        ds = xr.Dataset()
        with click.progressbar(
                udate_files,
                label=f'Processing {date:%Y-%m-%d} files to l1a:',
                item_show_func=lambda a:a
        ) as files:
            for fn in files:
                dst = starpas.data.read_raw(fn, config=config)
                ds = xr.merge((ds,dst),compat="no_conflicts",join="outer")

        fname_info = parse.parse(
            config["fname_raw"],
            os.path.basename(fn)
        ).named
        fname_info.update({
            "table": "complete",
            "campaign": config["campaign"],
            "resolution": "20Hz",
            "datalvl": "l1a",
            "sfx": "nc"
        })
        outfile = os.path.join(output_path,
                            "{dt:%Y/%m/}",
                            config['fname_out'])
        outfile = outfile.format_map(fname_info)

        starpas.data.to_netcdf(ds, fname=outfile)
    # add readme to output path
    fname_readme = os.path.join(
        importlib.resources.files("starpas"),
        "share/README.l1a.md"
    )
    shutil.copy2(fname_readme, output_path)


@cli.command("l1a2l1b")
@click.argument("ship_path",nargs=1, type=click.Path(dir_okay=True, exists=True))
@click.argument("input_files", nargs=-1)
@click.argument("output_path", nargs=1)
@click.option("--config", "-c", type=click.Path(dir_okay=False, exists=True),
              help="Config file - will merge and override the default config.")
def l1a2l1b(
    ship_path:str,
    input_files,
    output_path: str,
    config
):
    config = _configure(config)
    starpas.utils.init_logger(config)

    ship_path = os.path.abspath(ship_path)

    with click.progressbar(
            input_files,
            label='Processing to l1b:',
            item_show_func=lambda a:a
    ) as files:
        for fn in files:
            fname_info = parse.parse(
                config["fname_out"],
                os.path.basename(fn)
            ).named
            fname_info.update({
                "campaign": config["campaign"],
                "resolution": "20Hz",
                "datalvl": "l1b",
                "sfx": "nc"
            })

            # load ship data
            try:
                dsship = pd.read_csv(
                    os.path.join(ship_path,config["fname_ship"].format_map(fname_info)),
                    **config["read_csv"]
                )
                dsship = dsship.to_xarray().rename({"index":"time"})
            except Exception as e:
                print(e)
                continue

            # load l1a data
            l1a = xr.load_dataset(fn)

            # process to l1b
            l1b = starpas.data.l1a2l1b(l1a, dsship, config=config)

            outfile = os.path.join(output_path,
                                   "{dt:%Y/%m/}",
                                   config['fname_out'])
            outfile = outfile.format_map(fname_info)

            starpas.data.to_netcdf(l1b, fname=outfile)

    # add readme to output path
    fname_readme = os.path.join(
        importlib.resources.files("starpas"),
        "share/README.l1b.md"
    )
    shutil.copy2(fname_readme, output_path)

@cli.command("check")
@click.option("--config", "-c", type=click.Path(dir_okay=False, exists=True),
              help="Config file - will merge and override the default config.")
def checktoday(config):
    config = _configure(config)
    starpas.utils.init_logger(config)

    now = dt.datetime.utcnow()
    start = dt.datetime(now.year,now.month,now.day,0,0,0)
    fnames = starpas.ftp.fnames_in_period(start, now)[:]
    fnames_ftp = starpas.ftp.check_ftp_files(fnames, config=config)

    print(f"For today there are {len(fnames_ftp)} files on the FTP server.")
    print(f"The last recorded file is {fnames_ftp[-1]}.")