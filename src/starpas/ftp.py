import ftplib
import os
import pandas as pd
import datetime as dt
import numpy as np
from io import BytesIO
import logging
import click

import starpas.utils

logger = logging.getLogger(__name__)

def fnames_last(period="10min"):
    now = dt.datetime.utcnow()
    delta = pd.Timedelta(period)
    return fnames_in_period(now-delta, now)

def fnames_missing(start_date, store_files):
    now = dt.datetime.utcnow()
    fnames = fnames_in_period(start_date, now)[:-1]
    fnames_stored = []
    for p,d,f in os.walk(store_files):
        fnames_stored += [fi for fi in f]

    fnames_ftp = [os.path.split(fi)[-1] for fi in fnames]

    missing_mask = [fi not in fnames_stored for fi in fnames_ftp]
    return np.array(fnames)[missing_mask]


def fnames_in_period(start_date, end_date):
    dates = pd.date_range(start_date, end_date, freq="1min", inclusive="left")
    fnames = [f"{d:%Y/%m/%d/%H/%Y%m%d%H%M}.txt" for d in dates]
    return fnames

def check_ftp_files(fnames, config=None):
    config = starpas.utils.merge_config(config)

    paths = np.unique([os.path.dirname(f) for f in fnames])
    fnames_on_ftp = []

    with ftplib.FTP(config["address"]) as ftp:
        ftp.login(user=config["user"], passwd=config["passwd"])
        first_data = True
        with click.progressbar(
                paths,
                label='Checking FTP Files:',
                item_show_func=lambda a: a
        ) as pathsi:
            for path in pathsi:
                try:
                    ftp.cwd("/" + path)
                except:
                    continue
                result = ftp.mlsd()
                for r in result:
                    if r[1]["type"] == "file":
                        fnames_on_ftp.append(os.path.join(path, r[0]).replace("\\", "/"))
    return fnames_on_ftp

def read_from_ftp(fnames, config=None, parse_data=True, store_files=False, check_ftp=True):
    config = starpas.utils.merge_config(config)

    if check_ftp:
        fnames = check_ftp_files(fnames, config)

    with ftplib.FTP(config["address"]) as ftp:
        ftp.login(user=config["user"], passwd=config["passwd"])
        first_data = True
        with click.progressbar(
            fnames,
            label='Downloading:',
            item_show_func=lambda a: a
        ) as files:
            for fname in files:
                r = BytesIO()
                logger.info(f"Try to retrieve {fname} ...")
                try:
                    ftp.retrbinary(f"RETR {fname}", r.write)
                    logger.info(f"... success!")
                except Exception:
                    logger.warning(f"... failed, {fname} not available!", exc_info=Exception)
                    continue
                r = r.getvalue().decode()

                if store_files:
                    outfile = os.path.join(store_files, fname)
                    os.makedirs(os.path.dirname(outfile), exist_ok=True)
                    with open(outfile, "w") as txt:
                        txt.write(r)
                if parse_data:
                    # split lines
                    result = r.split("\r\n")

                    # remove empty lines
                    result = np.array(result)[np.char.str_len(result) != 0]

                    # make data array
                    data_tmp = np.vstack(np.char.split(result, ';'))

                    if first_data:
                        data = data_tmp.copy()
                        first_data = False
                    else:
                        data = np.vstack((data, data_tmp))
        ftp.quit()

    return data if parse_data else None
