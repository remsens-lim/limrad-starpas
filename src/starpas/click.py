import os.path
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

    with click.progressbar(
            input_files,
            label='Processing to l1a:',
            item_show_func=lambda a:a
    ) as files:
        for fn in files:
            ds = starpas.data.read_raw(fn, config=config)
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
