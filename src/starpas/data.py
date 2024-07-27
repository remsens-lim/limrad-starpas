import os.path
from toolz import assoc_in
import numpy as np
import xarray as xr
import pandas as pd
import logging

import starpas.utils

logger = logging.getLogger(__name__)

def get_cfmeta(config=None):
    """Read global and variable attributes and encoding from cfmeta.json
    """
    config= starpas.utils.merge_config(config)
    # parse the json file
    cfdict = starpas.utils.read_json(config["file_cfmeta"])
    # get global attributes:
    gattrs = cfdict['attributes']
    # apply config
    gattrs = {k : v.format_map(config) for k, v in gattrs.items()}
    # get variable attributes
    d = starpas.utils.get_var_attrs(cfdict)
    # split encoding attributes
    vattrs, vencode = starpas.utils.get_attrs_enc(d)
    return gattrs, vattrs, vencode

def add_encoding(ds, vencode=None):
    """
    Set valid_range attribute and encoding to every variable of the dataset.

    Parameters
    ----------
    ds: xr.Dataset
        Dataset of any processing level. The processing level will be
        determined by the global attribute 'processing_level'.
    vencode: dict or None
        Dictionary of encoding attributes by variable name, will be merged with pyrnet default cfmeta. The default is None.

    Returns
    -------
    xr.Dataset
        The input dataset but with encoding and valid_range attribute.
    """
    # prepare netcdf encoding
    _, vattrs_default, vencode_default = get_cfmeta()

    # Add valid range temporary to encoding dict.
    # As valid_range is not implemented in xarray encoding,
    #  it has to be stored as a variable attribute later.
    for k in vencode_default:
        if "valid_range" not in vencode_default[k]:
            continue
        vencode_default = assoc_in(vencode_default,
                                   [k, 'valid_range'],
                                   vattrs_default['valid_range'])

    # merge input and default with preference on input
    if vencode is None:
        vencode = vencode_default
    else:
        a = vencode_default.copy()
        b = vencode.copy()
        vencode = {}
        for k in set(a)-set(b):
            vencode.update({k: a[k]})
        for k in set(a)&set(b):
            vencode.update({k: {**a[k], **b[k]}})
        for k in set(b)-set(a):
            vencode.update({k: b[k]})

    # add encoding to Dataset
    for k, v in vencode.items():
        for ki in [key for key in ds if key.startswith(k)]:
            ds[ki].encoding = v
        if "valid_range" not in vencode[k]:
            continue
        # add valid_range to variable attributes
        for ki in [key for key in ds if key.startswith(k)]:
            ds[ki].attrs.update({
                'valid_range': vencode[k]['valid_range']
            })

    # add time encoding
    ds["time"].encoding.update({
        "dtype": 'f8',
        "units": f"seconds since {np.datetime_as_string(ds.time.data[0], unit='D')}T00:00Z",
    })
    return ds

def update_coverage_meta(ds, timevar='time'):
    """Update global attributes related to geospatial and time coverage
    """
    duration = ds[timevar].values[-1] - ds[timevar].values[0]
    resolution = np.mean(np.diff(ds[timevar].values))
    now = pd.to_datetime(np.datetime64("now"))
    gattrs = {
        'date_created': now.isoformat(),
        'time_coverage_start': pd.to_datetime(ds[timevar].values[0]).isoformat(),
        'time_coverage_end': pd.to_datetime(ds[timevar].values[-1]).isoformat(),
        'time_coverage_duration': pd.to_timedelta(duration).isoformat(),
        'time_coverage_resolution': pd.to_timedelta(resolution).isoformat(),
    }

    if ("lat" in ds) and ("lon" in ds):
        gattrs.update({
            'geospatial_lat_min': np.nanmin(ds.lat.values),
            'geospatial_lat_max': np.nanmax(ds.lat.values),
            'geospatial_lat_units': 'degN',
            'geospatial_lon_min': np.nanmin(ds.lon.values),
            'geospatial_lon_max': np.nanmax(ds.lon.values),
            'geospatial_lon_units': 'degE',
        })

    ds.attrs.update(gattrs)
    return ds

def merge_ds(ds1, ds2, timevar="time"):
    """Merge two datasets along the time dimension.
    """
    logger.info("Merge with existing file.")

    ## merge both datasets
    ds_new = xr.merge((ds1, ds2))

    if isinstance(ds1.raw_files,str):
        ds1files = [ds1.raw_files]
    else:
        ds1files = ds1.raw_files

    raw_files = list(np.unique(ds1files + ds2.raw_files))
    # add global coverage attributes
    ds_new.attrs.update({"raw_files": raw_files})

    ds_new = update_coverage_meta(ds_new, timevar=timevar)

    # add encoding again
    ds_new = add_encoding(ds_new)
    return ds_new

def to_netcdf(ds, fname, timevar="time"):
    """xarray to netcdf, but merge if exist
    """
    # create directory if not exists
    os.makedirs(os.path.dirname(fname), exist_ok=True)

    # merge if necessary
    if os.path.exists(fname):
        ds1 = xr.load_dataset(fname)
        if ds.raw_files[0] not in ds1.raw_files:
            ds = merge_ds(ds1, ds, timevar=timevar)
        else:
            ds = ds1.copy(deep=True)
            logger.info("Skip file, time index already exists.")
        ds1.close()
        os.remove(fname)

    # save to netCDF4
    ds = update_coverage_meta(ds, timevar=timevar)
    ds.to_netcdf(fname)


def read_raw(fname, config=None, global_attrs=None):
    config = starpas.utils.merge_config(config)
    gattrs, vattrs, vencode = get_cfmeta(config)

    if global_attrs is not None:
        gattrs.update(global_attrs)

    with open(fname) as f:
        result = f.readlines()

    # strip end of line
    result = np.char.rstrip(result, '\n')

    # remove empty lines
    result = np.array(result)[np.char.str_len(result) != 0]

    # make data array
    data = np.vstack(np.char.split(result, ';'))

    # parse datetime
    dates = np.char.add(data[:, 2], '-')
    dates = np.char.add(dates, np.char.zfill(data[:, 1], 2))
    dates = np.char.add(dates, '-')
    dates = np.char.add(dates, np.char.zfill(data[:, 0], 2))
    dates = np.char.add(dates, 'T')
    dates = np.char.add(dates, np.char.zfill(data[:, 3], 2))
    dates = np.char.add(dates, ':')
    dates = np.char.add(dates, np.char.zfill(data[:, 4], 2))
    dates = np.char.add(dates, ':')
    dates = np.char.add(dates, np.char.zfill(data[:, 5], 2))
    dates = np.char.add(dates, '.')
    dates = np.char.add(dates, np.char.zfill(data[:, 6], 3))
    dates = dates.astype("datetime64[ns]")

    # guess and add missing milliseconds
    udates, counts = np.unique(dates.astype("datetime64"), return_counts=True)
    for count, date in zip(counts, udates):
        timedelta = np.arange(count) * (np.timedelta64(int(1e9), 'ns') / count)
        dates[dates == date] = dates[dates == date] + timedelta

    # parse lat lon
    latitude = data[:, 7].astype(float) * 1e-2
    latitude[data[:, 8] != "N"] *= -1

    longitude = data[:, 9].astype(float) * 1e-2
    longitude[data[:, 10] != "E"] *= -1

    # make Dataset
    ds = xr.Dataset(
        {
            "lat": ("time", latitude),
            "lon": ("time", longitude),
            "gps-speed": ("time", data[:, 11].astype(float)),
            "altitude": ("time", data[:, 12].astype(float)),
            "gps-satellites": ("time", data[:, 13].astype(np.ushort)),
            "gps-fixquality": ("time", data[:, 14].astype(np.ushort)),
            "ax": ("time", data[:, 15].astype(float)),
            "ay": ("time", data[:, 16].astype(float)),
            "az": ("time", data[:, 17].astype(float)),
            "gx": ("time", data[:, 18].astype(float)),
            "gy": ("time", data[:, 19].astype(float)),
            "gz": ("time", data[:, 20].astype(float)),
            "mx": ("time", data[:, 21].astype(float)),
            "my": ("time", data[:, 22].astype(float)),
            "mz": ("time", data[:, 23].astype(float)),
            "q0": ("time", data[:, 24].astype(float)),
            "qx": ("time", data[:, 25].astype(float)),
            "qy": ("time", data[:, 26].astype(float)),
            "qz": ("time", data[:, 27].astype(float)),
            "yaw": ("time", data[:, 28].astype(float)),
            "pitch": ("time", data[:, 29].astype(float)),
            "roll": ("time", data[:, 30].astype(float)),
            "temperature": ("time", data[:, 31].astype(float)+273.15),
            "pressure": ("time", data[:, 32].astype(float)*100),
        },
        coords={
            "time": ("time", dates)
        }
    )

    now = pd.to_datetime(np.datetime64("now"))
    gattrs.update({
        "raw_files": [os.path.basename(fname),],
        'processing_level': 'l1a',
        'product_version': starpas.__version__,
        'history': f'{now.isoformat()}: Generated level l1a  by mordor version {starpas.__version__}; ',
    })
    ds.attrs.update(gattrs)

    # add global coverage attributes
    ds = update_coverage_meta(ds, timevar="time")

    # add attributes to Dataset
    for k, v in vattrs.items():
        if k not in ds.keys():
            continue
        # iterate over suffixed variables
        for ki in [key for key in ds if key.startswith(k)]:
            ds[ki].attrs.update(v)

    # add encoding to Dataset
    ds = add_encoding(ds, vencode)
    return ds
