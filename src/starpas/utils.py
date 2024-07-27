import numpy as np
import pandas as pd
from scipy import signal
from scipy.interpolate import interp1d
from scipy.optimize import minimize_scalar
from scipy.stats import circmean
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import logging
import importlib
import os
import jstyleson as json
from toolz import keyfilter
from operator import itemgetter
from addict import Dict as adict

logger = logging.getLogger(__name__)
def read_json(fpath: str, *, object_hook: type = adict, cls = None) -> dict:
    """ Parse json file to python dict.
    """
    with open(fpath,"r") as f:
        js = json.load(f, object_hook=object_hook, cls=cls)
    return js

def pick(whitelist: list[str], d: dict) -> dict:
    """ Keep only whitelisted keys from input dict.
    """
    return keyfilter(lambda k: k in whitelist, d)

def omit(blacklist: list[str], d: dict) -> dict:
    """ Omit blacklisted keys from input dict.
    """
    return keyfilter(lambda k: k not in blacklist, d)

def get_var_attrs(d: dict) -> dict:
    """
    Parse cf-compliance dictionary.

    Parameters
    ----------
    d: dict
        Dict parsed from cf-meta json.

    Returns
    -------
    dict
        Dict with netcdf attributes for each variable.
    """
    get_vars = itemgetter("variables")
    get_attrs = itemgetter("attributes")
    vattrs = {k: get_attrs(v) for k,v in get_vars(d).items()}
    for k,v in get_vars(d).items():
        vattrs[k].update({
            "dtype": v["type"],
            "gzip":True,
            "complevel":6
        })
    return vattrs

def get_attrs_enc(d : dict) -> (dict,dict):
    """ Split variable attributes in attributes and encoding-attributes.
    """
    _enc_attrs = {
        "scale_factor",
        "add_offset",
        "_FillValue",
        "dtype",
        "zlib",
        "gzip",
        "complevel",
        "calendar",
    }
    # extract variable attributes
    vattrs = {k: omit(_enc_attrs, v) for k, v in d.items()}
    # extract variable encoding
    vencode = {k: pick(_enc_attrs, v) for k, v in d.items()}
    return vattrs, vencode

def get_default_config():
    """
    Get starpas default config
    """
    fn_config = os.path.join(
        importlib.resources.files("starpas"),
        "share/starpas_config.json"
    )
    default_config = read_json(fn_config)

    # expand default file paths
    for key in default_config:
        if key.startswith("file"):
            default_config.update({
                key: os.path.join(
                    importlib.resources.files("starpas"),
                    default_config[key]
                )
            })
    return default_config

def merge_config(config):
    """
    Merge config dictionary with MORDOR default config
    """
    default_config = get_default_config()
    if config is None:
        config = default_config
    else:
        config = {**default_config, **config}
    return config

def init_logger(config):
    """
    Initialize Logging based on MORDOR config
    """
    config = merge_config(config)
    fname = os.path.abspath(config["file_log"])

    # logging setup
    logging.basicConfig(
        filename=fname,
        encoding='utf-8',
        level=logging.DEBUG,
        format='%(asctime)s %(name)s %(levelname)s:%(message)s'
    )

def _f_interp(dt,f):
    dt=dt.astype('datetime64[ns]').astype(int)
    return f(dt)

def time2int(x):
    return x.astype('datetime64[ns]').astype(int)

def calc_attitude_angle(rp, degrees=True, axis=1):
    if degrees:
        rp =np.deg2rad(rp)
    cosrp = np.cos(rp)
    dangl = np.arccos(np.prod(cosrp, axis=axis))
    if degrees:
        dangl = np.rad2deg(dangl)
    return dangl

def get_rate(time, data, res=10, axis=0):
    ores = time2int(np.diff(time).mean())
    sos = signal.butter(10, np.rint((ores * 1e-9) ** (-1)),
                        btype='lowpass', output='sos', fs=1000)
    newtime = pd.date_range(time[0], time[-1], freq=f'{res}ms')
    newtime = newtime.astype('datetime64[ns]').astype(int)
    
    f_data = interp1d(time.astype('datetime64[ns]').astype(int),
                    data,
                    axis=axis,
                    kind='cubic',
                    bounds_error=False,
                    fill_value=np.nan,
                    assume_sorted=True,
                    )
    idata = f_data(newtime)
    
    
    ddata = np.diff(idata, axis=axis)
    rate = ddata/(res*1e-3)
    rate = signal.sosfiltfilt(sos, rate, axis=axis)

    rate_time = newtime[:-1] + (0.5*res*1e6)

    f_rate = interp1d(rate_time,
                     rate,
                     axis=axis,
                     kind='cubic',
                     bounds_error=False,
                     fill_value=np.nan,
                     assume_sorted=True,
                    )
    frate = lambda x: _f_interp(x, f_rate)
    return frate

def get_tangential_accel(time, data, radius, res=10):
    ores = time2int(np.diff(time).mean())
    sos = signal.butter(res, np.rint((ores * 1e-9) ** (-1)),
                        btype='lowpass', output='sos', fs=1000)
    time=time2int(time)
    data[data>180] -= 360
    ddata = np.diff(data)
    adiff = np.abs(ddata)
    idx = adiff>180
    ddata[idx] = adiff[idx]-360
    drate = np.diff(ddata)
    accel = drate / (np.diff(time)[1:]*1e-9)**2
    accel = signal.sosfiltfilt(sos,accel,axis=0)
    # degrees/seconds**2 to m/seconds**2
    accel = np.deg2rad(accel)*radius 
    # accel_time = newtime[1:-1]
    accel_time = time[1:-1]
    f_accel = interp1d(accel_time,
                      accel,
                      kind='cubic',
                      bounds_error=False,
                      fill_value=np.nan,
                      assume_sorted=True,
                     )
    faccel = lambda x: _f_interp(x, f_accel)
    return faccel

def get_rate_accel(time, data, res=10, axis=0):
    ores = time2int(np.diff(time).mean())
    sos = signal.butter(res, np.rint((ores * 1e-9) ** (-1)),
                        btype='lowpass', output='sos', fs=1000)
    newtime = pd.date_range(time[0], time[-1], freq=f'{res}ms')
    newtime = newtime.astype('datetime64[ns]').astype(int)
    
    f_data = interp1d(time.astype('datetime64[ns]').astype(int),
                    data,
                    axis=axis,
                    kind='cubic',
                    bounds_error=False,
                    fill_value=np.nan,
                    assume_sorted=True,
                    )
    idata = f_data(newtime)
    
    
    ddata = np.diff(idata, axis=axis)
    rate = ddata/(res*1e-3)
    rate = signal.sosfiltfilt(sos, rate, axis=axis)

    rate_time = newtime[:-1] + (0.5*res*1e6)

    f_rate = interp1d(rate_time,
                     rate,
                     axis=axis,
                     kind='cubic',
                     bounds_error=False,
                     fill_value=np.nan,
                     assume_sorted=True,
                    )
    
    drate = np.diff(ddata,axis=0)
    accel = drate / (res*1e-3)**2
    accel = signal.sosfiltfilt(sos,accel,axis=0)
    accel_time = newtime[1:-1]

    f_accel = interp1d(accel_time,
                      accel,
                      axis=axis,
                      kind='cubic',
                      bounds_error=False,
                      fill_value=np.nan,
                      assume_sorted=True,
                     )
    
    frate = lambda x: _f_interp(x, f_rate)
    faccel = lambda x: _f_interp(x, f_accel)
    return frate, faccel

def get_lag(series,reference,step=1,offset=0,plot=False):
    """ Use cross-corelation to find (time) lag of a series versus a reference.
    Returns dt, so that series(t-dt) has maximum correlation to reference(t).
    """
    noffset = int(np.round(np.array(offset).astype(float)/np.array(step).astype(float),0))
    corr = signal.correlate(series[noffset:],reference)
    lags = signal.correlation_lags(len(series), len(reference))
    imax = np.argmax(corr)
    
    if plot:
        fig,ax = plt.subplots(1,1)
        peaks, peakprops = signal.find_peaks(corr,prominence=(None, None))
        peakimax = np.argmax(peakprops['prominences'])
        dt = step*lags.astype(float)*1e-3
        ax.plot(dt,corr)
        ax.plot(dt[imax],corr[imax],ms=10,marker='x')
        ax.plot(dt[peaks[peakimax]],corr[peaks[peakimax]],ls='',ms=10,marker='.')
        # ax.plot(dt[peaks],corr[peaks],ls='',ms=10,marker='.')
        ax.set_xlim([-100,100])
        ax.grid(True)
    
    if np.all(np.isnan(corr)):
        return np.nan
    else:
        return step*lags[imax] + offset

def circ_corrcoef(x, y, deg=True):
    """
    Calculate circular correlation coefficient
    """
    if deg:
        x = np.deg2rad(x)
        y = np.deg2rad(y)
    sx = np.sin(x-circmean(x))
    sy = np.sin(y-circmean(y))

    r = np.sum(sx*sy) / np.sqrt(np.sum(sx**2)*np.sum(sy**2))
    return r

def accel2rp(Xaccl,Yaccl,Zaccl):
    """ Assuming right handed rotation along axis, rotation order XYZ """
    roll = np.arctan2(Yaccl,Zaccl)
    pitch = - np.arctan2(Xaccl,np.sqrt(Yaccl**2+Zaccl**2))
    
    roll = np.rad2deg(roll)
    pitch = np.rad2deg(pitch)
    
    roll = np.where(roll < 90, roll, roll-180)
    roll = np.where(roll >-90, roll, roll+180)
    pitch = np.where(pitch < 90, pitch, pitch-180)
    pitch = np.where(pitch >-90, pitch, pitch+180)

    return roll, pitch


def xyz2rp(xyz,rotation=(1,1), degrees=True):
    """
    Calculate pitch and roll angle of a Cartesian vector assuming heading aligned
    coordinate system
      * ships bow points along x-axis
      * pitch -> left-hand rotation around y-axis-> positive if bow is up
      * roll -> right-hand rotation around x-axis -> positive if starboard is down
      * Coordinate system:
        X: North or ships bow,
        Y: West or ships port-side
        Z: upward

    Parameters
    ----------
    xyz: list of len(3), or numpy.array of shape(N,3)
        karthesian coordinates of ships normal vector
    rotation: list of len(2)
        rotation definition of roll, pitch. 1= right-hand, -1 = left hand

    Returns
    -------
    rp: numpy.array of shape (N,2)
        axis [:,0] roll angle [degrees] (positive if starboard is down)
        axis [:,1] pitch angle [degrees] (positive if bow is up)
    """
    xyz = np.array(xyz)
    if len(xyz.shape) == 1:
        xyz = xyz[np.newaxis, :]
    # calculate roll and pitch as seen from the ship
    r = np.arctan2(xyz[:, 1], 
                   xyz[:, 2])
    p = -np.arctan2(xyz[:, 0],
                   np.linalg.norm(xyz[:,[1,2]], axis=1))
    
    # roll valid range is in principle -180 to 180
    # if coordinate system is flipped (zdirection is downward)
    # we will receive r values around 180,
    # to use this function flexible regardless if coordinate system
    # is upright or flipped, we transform r values <-90 and >90
    # assuming a flipped coordinate system.
    # Therefore, this function is only valid for roll angles <90deg
    r = np.where(r < np.pi/2, r, -(r-np.pi))
    r = np.where(r > -np.pi/2, r, -(r+np.pi))
    
    # restrict pitch to its valid values [-90,90]
    p = np.where(p < np.pi/2, p, p-np.pi)
    p = np.where(p > -np.pi/2, p, p+np.pi)             
    
    # unit conversion
    if degrees:
        r = np.rad2deg(r)
        p = np.rad2deg(p)
    
    rp = np.vstack([r, p]).T

    return rp * np.array(rotation)

def rpy2xyz_rate_accel(rpy,time, heave=None, res=10, position=(-11,4.07,-15.8)):
    """
    calculate vector, rate and acceleration in a right handed system
    """
    def _f_interp(dt,f):
        dt=dt.astype('datetime64[ns]').astype(int)
        return f(dt)

    def time2int(x):
        return x.astype('datetime64[ns]').astype(int)

    position = np.array(position)
    ores = time2int(np.diff(time).mean())
    sos = signal.butter(10, np.rint((ores * 1e-9) ** (-1)),
                        btype='lowpass', output='sos', fs=1000)

    
    if heave is not None:
        position = np.repeat(position[np.newaxis,:],
                             repeats=rpy.shape[0],
                             axis=0)
        position[:,2] += heave
    
    XYZa = rpy2xyz(rpy,
                   vector=np.array(position),
                   rotation=(1,1,1),
                   degrees=True)
    
    
    newtime = pd.date_range(time[0],time[-1],freq=f'{res}ms')
    newtime = newtime.astype('datetime64[ns]').astype(int)

    f_XYZ = interp1d(time.astype('datetime64[ns]').astype(int),
                    XYZa,
                    axis=0,
                    kind='cubic',
                    bounds_error=False,
                    fill_value=np.nan,
                    assume_sorted=True,
                    )
    XYZ = f_XYZ(newtime)

    dXYZ = np.diff(XYZ,axis=0)
    rate = dXYZ/(res*1e-3)
    rate = signal.sosfiltfilt(sos, rate, axis=0)

    rate_time = newtime[:-1]+(0.5*res*1e6)

    f_rate = interp1d(rate_time,
                     rate,
                     axis=0,
                     kind='cubic',
                     bounds_error=False,
                     fill_value=np.nan,
                     assume_sorted=True,
                    )

    
    # drate = np.diff(rate,axis=0)
    drate = np.diff(dXYZ,axis=0)
    accel = drate / (res*1e-3)**2
    accel = signal.sosfiltfilt(sos,accel,axis=0)

    # accel_time = rate_time[:-1]+(0.5*res*1e6)
    accel_time = newtime[1:-1]

    f_accel = interp1d(accel_time,
                      accel,
                      axis=0,
                      kind='cubic',
                      bounds_error=False,
                      fill_value=np.nan,
                      assume_sorted=True,
                     )

    fXYZ = lambda x: _f_interp(x, f_XYZ)
    frate = lambda x: _f_interp(x, f_rate)
    faccl = lambda x: _f_interp(x, f_accel)
    
    return fXYZ, frate, faccl

def rpy2xyz(rpy,vector = np.array([0, 0, 1]), rotation=(1,1,1), degrees=True):
    """
    Calculate Cartesian coordinates of ships normal vector (x=0,y=0,z=1) if rotated
    by the angles roll, pitch and yaw.
      * ships bow points along x-axis
      * pitch -> rotation around y-axis
          * right-hand -> positive if bow is down
          * left-hand -> positive if bow is up
      * roll -> right-hand rotation around x-axis -> positive if starboard is down
      * yaw -> left-hand rotation around z-axis -> positive if bow moves clockwise
                or positive from north (if x-axis points towards north)
      * Coordinate system:
        X: North or ships bow,
        Y: West or ships port-side
        Z: upward

    Parameters
    ----------
    rpy: list/tuple of len(3) or numpy.array of shape(N,3)
        Roll, pitch, and yaw angles in degrees [roll, pitch, yaw]
        or np.array([[roll1, pitch1, yaw1], ..., [rollN, pitchN, yawN]])
    rotation: list,len(3)
        rotation definition of roll, pitch, and yaw. 1 equals right-hand (counter-clockwise) rule. -1 equals left-hand rule
        default is [1,1,1] ( all right-hand rotation )
    degrees: bool
        If True, the unit of angles in rpy is assumed to be degrees, else rad.
        The default is True.

    Returns
    -------
    xyz: np.array of shape (N,3)
        Cartesian coordinates x,y,z
    """

    # ensure shape
    rpy = np.array(rpy)
    if len(rpy.shape) == 1:
        rpy = rpy[np.newaxis, :]

    # initialize normal vector
    

    # sort to match order of rotation
    ypr = rpy[:, [2, 1, 0]]

    # as rotation will be applied as right-hand rotation later,
    # invert yaw and pitch, to achieve left-hand rotation of the same.
    ypr = ypr * np.array(rotation[::-1])

    # calculate right-hand euler rotation matrix
    r = R.from_euler("ZYX", ypr, degrees=degrees)

    # apply rotation and return
    return r.apply(vector)

def compensate_yaw(yaw, rp, vector=np.array([0,0,1]), rotation=(1,1,1), degrees=True):
    """
    Rotate given roll and pitch, so that output roll and pitch are as if yaw=0
    
    Parameters
    ----------
    yaw: float
        Yaw angle - rotation around z-axis, defined by rotation[2]
    rp: numpy.array([N,2])
        roll (column 0, rotation around x-axis defined by rotation[0])
        pitch (column 1, rotation around y-axis defined by rotation[1])
    rotation: tuple(3)  of 1 or -1, default=(1,1,1)
        Rotation definition of (Roll, Pitch, Yaw).
            * 1: right-hand rotation around axis,
            * -1: left-hand rotation around axis,
        with:
            * Roll - rotation around x-axis
            * Pitch - rotation around y-axis
            * Yaw - rotation around z-axis
        Coordinate system:
            Y-axis positive to the left, looking along x-axis,
            Z-axis pointing upward
    degrees: bool, default=True
        Whether or not angles units are degrees or not.
    
    Returns
    -------
    rp0: numpy.array([N,2])
        roll and pitch as if yaw is 0 -> rotated by -1*yaw
    """
    # adding yaw to compensate
    rpy = np.hstack((rp,np.ones((rp.shape[0],1))*yaw))
    # calculate normal vector with added negative yaw
    xyz = rpy2xyz(rpy, vector=vector, rotation=rotation, degrees=degrees)
    
    # Transform  pitch/roll, so that yaw==0 so added yaw is compensated.
    # Therefore, pitch/roll are 'rotated' in -1*yaw direction.
    rp0 = xyz2rp(xyz, rotation=rotation[:2], degrees=degrees)
    
    # return rotated roll and pitch
    return rp0

def estimate_dangle(ds,vector=np.array([0,0,1]), rotation=(1,1,1), degrees=True):
    """
    Derive offset alignment angles of instrument setup on a platform. The ship data
    corresponds to the platform alignment angles. Offset roll and pitch are defined as
    instrument and ship heading are alignet.
    The resulting yaw angle of the instrument describes the setup offset of heading of
    instrument and ship.
    
    Parameters
    ----------
    ds: xarray.Dataset
        Dataset have to include:
            * roll_inst:  Instrument Roll
            * pitch_inst: Instrument Pitch
            * roll_ship:  Ship Roll
            * pitch_ship: Ship Pitch
    rotation: tuple(3)  of 1 or -1, default=(1,1,1)
        Rotation definition of (Roll, Pitch, Yaw).
            * 1: right-hand rotation around axis,
            * -1: left-hand rotation around axis,
        with:
            * Roll - rotation around x-axis
            * Pitch - rotation around y-axis
            * Yaw - rotation around z-axis
        Coordinate system:
            Y-axis positive to the left, looking along x-axis,
            Z-axis pointing upward
    degrees: bool, default=True
        Whether or not angles units are degrees or not.


    Returns
    -------
    (OffsetRoll, OffsetPitch): (float, float)
        Offset roll and pitch angles from platform to guvis normal vector. [degrees]
    Yaw_Guvis: float
        Yaw Angle of the GUVis accelerometer. Positive clockwise from north. [degrees]
    """

    def _test_yaw(yaw_test, xyz_ship, roll_guvis, pitch_guvis, vector,rotation, degrees=True):
        rpy_guvis = np.vstack([roll_guvis,
                               pitch_guvis,
                               np.ones(len(roll_guvis)) * yaw_test]).T
        xyz_guvis = rpy2xyz(rpy_guvis,vector=vector, rotation=rotation, degrees=degrees)

        dot = np.sum(xyz_ship * xyz_guvis, axis=1)
        dangles = np.arccos(dot)
        return np.mean(dangles)


    # calculate mean sampling frequency
    freq = 1e3 / (np.diff(ds.time.data)).astype('timedelta64[ms]').astype(int)
    mean_freq = np.mean(freq)

    # smooth out ripples using 2sec rolling mean
    if mean_freq>1:
        ds = ds.rolling(time=int(np.round(mean_freq, 0))*2, center=True).mean().dropna("time")

    
    # ship in heading aligned coordinate system
    rp0_head_ship = np.vstack([ds.roll_ship.data,
                          ds.pitch_ship.data,
                          np.zeros(ds.time.size)]).T # heading aligned, setting yaw to zero
    # carthesian coordinates of normal vector
    xyz_head_ship = rpy2xyz(rp0_head_ship,
                            vector=vector,
                            rotation=rotation,
                            degrees=degrees)

    # find yaw offset, between ship and instrument
    res = minimize_scalar(_test_yaw,
                          bounds=[0,360],
                          args=(xyz_head_ship, ds.roll_inst.data, ds.pitch_inst.data,vector,rotation, degrees),
                          method='bounded')

    yaw_inst = float(res.x)
    
    # transform  instrument pitch/roll, so that yaw==0
    # after this transformation, roll/pitch of ship and instrument can be compared directly,
    # as they are pointing in the same direction
    rp0_head_inst = compensate_yaw(yaw_inst, 
                                   np.vstack([ds.roll_inst.data,
                                              ds.pitch_inst.data]).T,
                                   vector=vector,
                                   rotation=rotation,
                                   degrees=degrees)
    

    # calculate misalignment roll and pitch angles between ship and instrument
    # For this, we compare adjusted platform roll and pitch to
    # the instrument angles, but avoiding peaks of roll and pitch angles
    # as they are prone to be more errorneouse (stronger forces, as the instrument
    # is not in the rotation axis)
    
    # 1. step: Find time index between peeks of roll or pitch
    # (width of peaks is assumed minimum 1 second)
    roll_peaks, roll_peaks_res = signal.find_peaks(ds.roll_inst.data, width=[mean_freq])
    pitch_peaks, pitch_peaks_res = signal.find_peaks(ds.pitch_inst.data, width=[mean_freq])
    roll_left_ips = np.round(roll_peaks_res['left_ips'], 0).astype(int)
    roll_right_ips = np.round(roll_peaks_res['right_ips'], 0).astype(int)
    pitch_left_ips = np.round(pitch_peaks_res['left_ips'], 0).astype(int)
    pitch_right_ips = np.round(pitch_peaks_res['right_ips'], 0).astype(int)
    idx_half_peak_roll = np.unique(np.concatenate((roll_left_ips,
                                                   roll_right_ips), axis=0))
    idx_half_peak_pitch = np.unique(np.concatenate((pitch_left_ips,
                                                    pitch_right_ips), axis=0))

    delta_roll = np.mean(rp0_head_inst[idx_half_peak_roll, 0] - ds.roll_ship.data[idx_half_peak_roll])
    delta_pitch = np.mean(rp0_head_inst[idx_half_peak_pitch, 1] - ds.pitch_ship.data[idx_half_peak_pitch])
    delta_roll = float(delta_roll)
    delta_pitch = float(delta_pitch)

    # return roll and pitch offset (with aligned heading of ship and instrument)
    # Therefore, the instrument alignment can be described by 
    # roll_inst = roll_ship + delta_roll
    # pitch_inst = pitch_ship + delta_pitch
    # The returned value of yaw_inst describes the rotation of the instrument relative to the
    # ship heading, and is only for control
    return (delta_roll, delta_pitch), yaw_inst