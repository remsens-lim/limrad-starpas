import os.path
from toolz import assoc_in
import numpy as np
import xarray as xr
import pandas as pd
import logging
import jstyleson as json
from scipy import signal
from scipy.interpolate import interp1d

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
    setdifftime = np.setdiff1d(ds1[timevar].values,ds2[timevar].values)
    if len(setdifftime) == 0:
        # no additional time samples are in the old ds1
        logger.info("Overwrite existing file.")
        return ds2
    logger.info("Merge with existing file.")

    ## overwrite non time dependent variables
    overwrite_vars = [ v for v in ds1 if timevar not in ds1[v].dims ]

    ## just use data unique in the old ds1
    # (replace by new ds2 samples if samples overlap)
    ds1 = ds1.interp(time=setdifftime)

    ## merge both datasets
    ds_new=ds1.merge(ds2,
                     compat='no_conflicts',
                     join='outer',
                     overwrite_vars=overwrite_vars)

    # add global coverage attributes
    ds_new.attrs.update({'merged':1})

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

    # Fix: sometimes only the last two year digits are reported
    # in this case, add "20"
    ylen = np.char.str_len(data[:,2])
    data[ylen==2,2] = np.char.add("20",data[ylen==2,2])

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
            "gps-speed": ("time", 0.514444 * data[:, 11].astype(float)), #(m/s)
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
        'history': f'{now.isoformat()}: Generated level l1a  by starpas version {starpas.__version__}; ',
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

def pad_gaps(l1a,pad='10min',gap='10min'):
    """
    Remove values within 'pad' length from next gap of >'gap' length from dataset.
    
    :param l1a: l1a xarray dataset
    :param pad: pad length string in pandas period string
    :param gap: gap length string in pandas period string
    """
    dtime = np.diff(l1a.time.values)
    igap = np.array([-1] + list(np.argwhere(dtime>pd.Timedelta(gap)).ravel()) + [l1a.time.size])

    newtimes = []
    for istart,iend in zip(igap[:-1]+1,igap[1:]):
        if iend-istart<60:
            continue
        time_sel = l1a.time.isel(time=slice(istart,iend)).values  
        time_sel = l1a.time.sel(time=slice(time_sel[0]+pd.Timedelta(pad),time_sel[-1]-pd.Timedelta(pad)))
        newtimes += list(time_sel.values)
    
    return l1a.sel(time=np.array(newtimes))

def smooth_raw(l1a, swindow=20):
    l1a.ax.values = np.convolve(l1a.ax.values,np.ones(swindow),'same')/swindow
    l1a.ay.values = np.convolve(l1a.ay.values,np.ones(swindow),'same')/swindow
    l1a.az.values = np.convolve(l1a.az.values,np.ones(swindow),'same')/swindow

    l1a.gx.values = np.convolve(l1a.gx.values,np.ones(swindow),'same')/swindow
    l1a.gy.values = np.convolve(l1a.gy.values,np.ones(swindow),'same')/swindow
    l1a.gz.values = np.convolve(l1a.gz.values,np.ones(swindow),'same')/swindow
    
    l1a.mx.values = np.convolve(l1a.mx.values,np.ones(swindow),'same')/swindow
    l1a.my.values = np.convolve(l1a.my.values,np.ones(swindow),'same')/swindow
    l1a.mz.values = np.convolve(l1a.mz.values,np.ones(swindow),'same')/swindow
    return l1a

def smooth_ship(shipds,swindow=20):
    shipds["roll"].values = np.convolve(shipds["roll"].values,np.ones(swindow),'same')/swindow
    shipds["pitch"].values = np.convolve(shipds["pitch"].values,np.ones(swindow),'same')/swindow
    shipds["heading"].values = np.convolve(shipds["heading"].values,np.ones(swindow),'same')/swindow
    shipds["heave"].values = np.convolve(shipds["heave"].values,np.ones(swindow),'same')/swindow
    return shipds

def correct_lag(l1a, shipds, config):
    config = starpas.utils.read_json("/Users/witthuhn/Data/bowtie/starpas/starpas_config.json")
    config = starpas.utils.merge_config(config)
    
    l1a_raw = xr.load_dataset("/Users/witthuhn/Data/bowtie/starpas/l1a/2024/08/2024-08-23_starpas_bowtie_20Hz_l1a.nc")
    shipds = pd.read_csv(
        "/Users/witthuhn/Data/bowtie/seapath/20240823_dship_data_10hz.dat",
        **config["read_csv"]
    )
    shipds = shipds.to_xarray().rename({"index":"time"})
    
    
    # interpolate starpas to ship and drop nan values
    shipds = shipds.dropna("time")
    l1a = l1a_raw.interp_like(shipds).dropna("time")
    shipds = shipds.interp_like(l1a)
    
    # smooth rawdata
    swindow = 20
    sax = np.convolve(l1a.ax.values,np.ones(swindow),'same')/swindow
    say = np.convolve(l1a.ay.values,np.ones(swindow),'same')/swindow
    saz = np.convolve(l1a.az.values,np.ones(swindow),'same')/swindow
    
    # apply axis mapping and convention
    xyz_index = config["axis_mapping"]["xyz_index"]
    xyz_direction = np.array(config["axis_mapping"]["xyz_direction"])[None,:]
    acc = np.vstack([sax, say, saz]).T
    acc = acc[:, xyz_index] * xyz_direction
    
    # roll /pitch from ship
    rpy = np.vstack((
        np.convolve(shipds["roll"].values,np.ones(swindow),'same')/swindow,
        np.convolve(shipds["pitch"].values,np.ones(swindow),'same')/swindow,
        np.convolve(shipds["heading"].values,np.ones(swindow),'same')/swindow
    )).T
    
    ship_heave = -1.*shipds["heave"].values # invert heave, as our reference system points downward
    ship_heave  = np.convolve(ship_heave,np.ones(swindow),'same')/swindow
    
    # calculate external accelerations induced by the ship movement
    position = np.array(config["position"]).astype(float)
    _,_,spACCEL  = starpas.utils.rpy2xyz_rate_accel(
        rpy,
        time=shipds.time.values,
        heave=ship_heave,
        position=position,
    )
    
    ship_acc_vector = spACCEL(l1a.time.values)
    ship_acc_vector *= 1e3/9.81 # m s-2 -> mg
    ship_acc_vector[:,2] += 1000 # add gravitation
    
    
    ### get lag between ship and sensor
    acc_norm = np.linalg.norm(acc[:,:],axis=1)
    ship_norm = np.linalg.norm(ship_acc_vector[:,:],axis=1)
    
    init_offset = 100
    chunks = 100
    Nsamples = int(float(acc.shape[0]-4*init_offset)/float(chunks))
    xlags = []
    lags = []
    step = np.unique(np.diff(shipds.time.values))[0].astype("timedelta64[ns]")
    for ic in range(chunks):
        xlags.append(2*init_offset+ ic*Nsamples + Nsamples//2)
        lags.append(init_offset + get_lag(
            acc_norm[2*init_offset+ic*Nsamples:2*init_offset+(1+ic)*Nsamples],
            ship_norm[init_offset+ic*Nsamples:3*init_offset+(1+ic)*Nsamples],
            step=1
        ))
    
    xlags = np.array([0]+xlags+[acc.shape[0]])*step.astype(float)
    lags = np.array([lags[0]] + lags + [lags[-1]])
    
    itimes = (l1a_raw.time.values-l1a.time.values[0]).astype("timedelta64[ns]").astype(float)
    lag = np.interp(itimes, xlags, lags)*step
    
    l1a_raw = l1a_raw.assign_coords({"new_time":("time",l1a_raw.time.values+lag)})
    l1a_raw = l1a_raw.drop_vars("time").rename({"new_time":"time"})
    return l1a_raw


def get_lag(time,norm1,norm2,gap='10min',window=20*60*10):
    dtime = np.diff(time)
    igap = np.array([-1] + list(np.argwhere(dtime>pd.Timedelta(gap)).ravel()) + [time.size])

    new = True
    times = []
    lags = []
    for istart,iend in zip(igap[:-1]+1,igap[1:]):
        if iend-istart<window:
            continue
        itime = time[istart:iend]
        inorm1 = norm1[istart:iend]
        inorm2 = norm2[istart:iend]
        jedges = np.array(list(np.arange(0,iend-istart,window))+[-1]).astype(int)
        for i,j in enumerate(jedges[:-1]):
            if jedges[i+1]-jedges[i]<int(window/2):
                continue
            ti = itime[jedges[i]+int(window/2)]
            n1 = inorm1[jedges[i]:jedges[i+1]]
            n2 = inorm2[jedges[i]:jedges[i+1]]
            mask = (~np.isnan(n1))*(~np.isnan(n2))
            c_tmp = 0
            lag = 0
            for lag_test in range(-10,10):
                c = np.corrcoef(n1[11:-11-lag_test],n2[11+lag_test:-11])[1,0]
                if 1-c < 1-c_tmp:
                    lag = lag_test
                    c_tmp=c
            if lag==-10 or lag==10: # limit lag to +-10
                continue
            times.append(ti)
            lags.append(lag)
            # print(ti,lag)
    # f_lag = interp1d(times,lags,kind="nearest")
    # return np.array(f_lag(time)).astype(int)
    return int(np.nanmean(lags))
            

def l1a2l1b(l1a, shipds, config=None):
    config = starpas.utils.merge_config(config)
    gattrs, vattrs, vencode = get_cfmeta(config)

    # smooth rawdata
    l1a = smooth_raw(l1a,swindow=20)
    l1a = pad_gaps(l1a,pad='10min',gap='10min')
    shipds = smooth_ship(shipds,swindow=20)
    
    # remove nan and interpolate ship to l1a
    l1a = l1a.dropna("time").where(~np.isnan(l1a.time),drop=True)
    shipds = shipds.interp_like(l1a).dropna("time")
    l1a = l1a.reindex_like(shipds)

    # apply axis mapping and convention
    xyz_index = config["axis_mapping"]["xyz_index"]
    xyz_direction = np.array(config["axis_mapping"]["xyz_direction"])[None,:]

    gyr = np.vstack([l1a.gx.values, l1a.gy.values, l1a.gz.values]).T
    gyr = gyr[:, xyz_index] * xyz_direction
    gyr *= -1 # invert for right handed sytem

    acc = np.vstack([l1a.ax.values, l1a.ay.values, l1a.az.values]).T
    acc = acc[:, xyz_index] * xyz_direction

    # roll /pitch from ship
    rpy = np.vstack((
        shipds["roll"].values, shipds["pitch"].values, 
        # np.zeros(l1a.time.size)
        shipds["heading"].values[0] - shipds["heading"].values
    )).T
        
    ship_heave = 1.*shipds["heave"].values # invert heave, as our reference system points downward

    # calculate external accelerations induced by the ship movement
    position = np.array(config["position"]).astype(float)
    # print(position)
    _,_,spACCEL  = starpas.utils.rpy2xyz_rate_accel(
        rpy,
        time= shipds.time.values,
        heave=ship_heave,
        position=position,
    )

    ship_acc_vector = spACCEL(l1a.time.values)
    ship_acc_vector *= 1e3/9.81 # m s-2 -> mg

    norm_ship = np.linalg.norm(ship_acc_vector-np.array([0,0,1000]),axis=1)
    norm_acc = np.linalg.norm(acc,axis=1)
    lag = get_lag(l1a.time.values,norm_ship,norm_acc)

    if lag>0:
        l1a = l1a.isel(time=slice(lag,None))
        acc = acc[lag:]
        gyr = gyr[lag:]
        rpy = rpy[:-lag,:]
        shipds = shipds.isel(time=slice(None,-lag))
        ship_acc_vector = ship_acc_vector[:-lag]
        ship_heave = ship_heave[:-lag]
    elif lag<0:
        lag *= -1
        l1a = l1a.isel(time=slice(None,-lag))
        acc = acc[:-lag]
        gyr = gyr[:-lag]
        rpy = rpy[lag:,:]
        shipds = shipds.isel(time=slice(lag,None))
        ship_acc_vector = ship_acc_vector[lag:]
        ship_heave = ship_heave[lag:]

    l1a=l1a.assign_coords({"time":shipds.time})

    acc_corr = acc + ship_acc_vector

    # calculate roll and pitch from corrected accelerometer
    rpacc = -1.*starpas.utils.xyz2rp(acc_corr)

    # correct acceleration and add smoothed values in l1a
    l1a.ax.values = acc_corr[:,0]
    l1a.ay.values = acc_corr[:,1]
    l1a.az.values = acc_corr[:,2]

    l1a.gx.values = gyr[:,0]
    l1a.gy.values = gyr[:,1]
    l1a.gz.values = gyr[:,2]

    # update config, as axis mapping has been applied already
    config = {
        **config,
        "axis_mapping": {
            "xyz_index": [0,1,2],
            "xyz_direction": [1,1,1]
        }
    }

    # apply imufusion algorithm
    time, euler, istates, flags = starpas.utils.apply_imufusion(l1a, freq=config["imufusion"]["freq"], gap="10min", config=config)

    # calculate imufusion relying only on gyro
    # gyrconfig = dict(imufusion={
    #     "use_mag": False, 
    #     "gain": 0, 
    #     "gyro_range": 2000, # gyroscope-range (deg/s)
    #     "acc_reject": 0, # reject accelerometer if deviates more than X deg from algo output
    #     "mag_reject": 5, # reject magnetometer if deviates more than X deg from algo output
    #     "recovery": 120 # recovery trigger period in seconds (e.g. maximum time algo can depend only on gyro)
    # })
    # _,eulergyr,_,_ = starpas.utils.apply_imufusion(l1a, freq=config["imufusion"]["freq"], gap="10min", filtfreq=1/60., config={**config,**gyrconfig})
    # _,eulergyr,_,_ = starpas.utils.apply_imufusion(l1a, freq=config["imufusion"]["freq"], gap="10min", config={**config,**gyrconfig})

    # roll/pitch only from gyro
    rpgyr = starpas.utils.rate2angle(
        l1a.time.values,
        gyr,
    )
    for i in range(rpgyr.shape[1]):
        rpgyr[:,i] = starpas.utils.butter_highpass_filter(rpgyr[:,i],1./60.,10)

    #_,eulergyr,_,_ = starpas.utils.apply_imufusion(l1a, freq=config["imufusion"]["freq"], gap="10min", config={**config,**gyrconfig})


    # select same time for rpacc
    idx = np.isin(l1a.time.values,time)
    rpacc = rpacc[idx,:]
    rpgyr = rpgyr[idx,:]
    ship_acc_vector = ship_acc_vector[idx,:]

    shipds = shipds.sel(time=time)
    l1a = l1a.sel(time=time)

    l1b = xr.Dataset({
        "roll": ("time",euler[:,0]),
        "pitch": ("time", euler[:,1]),
        "yaw": ("time", euler[:,2]),
        "roll_acc": ("time", rpacc[:,0]),
        "pitch_acc": ("time", rpacc[:,1]),
        "yaw_acc": ("time", np.full(rpacc.shape[0],np.nan)),
        # "roll_gyr": ("time", eulergyr[:,0]),
        # "pitch_gyr": ("time", eulergyr[:,1]),
        # "yaw_gyr": ("time", eulergyr[:,2]),
        "roll_gyr": ("time", rpgyr[:,0]),
        "pitch_gyr": ("time", rpgyr[:,1]),
        "yaw_gyr": ("time",np.full(l1a.time.size,np.nan)),
        "fusion_states": (("time","states"), istates),
        "fusion_flags":(("time","flags"), flags.astype(bool)),
        "temperature": l1a.temperature,
        "pressure": l1a.pressure,
        "ax": ("time", l1a.ax.values),
        "ay": ("time", l1a.ay.values),
        "az": ("time", l1a.az.values),
        "ax_ship": ("time", ship_acc_vector[:,0]),
        "ay_ship": ("time", ship_acc_vector[:,1]),
        "az_ship": ("time", ship_acc_vector[:,2]),
        "gx": ("time", l1a.gx.values),
        "gy": ("time", l1a.gy.values),
        "gz": ("time", l1a.gz.values),
        "lat": l1a.lat,
        "lon": l1a.lon,
        "gps-speed": l1a["gps-speed"],
        "altitude": l1a.altitude,
        "gps-satellites": l1a["gps-satellites"],
        "gps-fixquality": l1a["gps-fixquality"],
        "roll_ship": ("time",shipds["roll"].values),
        "pitch_ship": ("time",shipds["pitch"].values),
        "yaw_ship": ("time",shipds["heading"].values),
        "heave_ship": ("time",shipds["heave"].values)
    }, coords={
        "time": ("time",time),
        "states": ("states",np.arange(6)),
        "flags": ("flags", np.arange(4))
    })


    now = pd.to_datetime(np.datetime64("now"))
    gattrs.update({
        'imufusion_axismapping': json.dumps(config["axis_mapping"]),
        'imufusion_settings': json.dumps(config["imufusion"]),
        'processing_level': 'l1b',
        'product_version': starpas.__version__,
        'history': f'{now.isoformat()}: Generated level l1b  by starpas version {starpas.__version__}; ',
    })
    l1b.attrs.update(gattrs)

    # add global coverage attributes
    l1b = update_coverage_meta(l1b, timevar="time")

    # add attributes to Dataset
    for k, v in vattrs.items():
        if k not in l1b.keys():
            continue
        # iterate over suffixed variables
        for ki in [key for key in l1b if key.startswith(k)]:
            l1b[ki].attrs.update(v)

    # add encoding to Dataset
    l1b = add_encoding(l1b, vencode)
    return l1b
