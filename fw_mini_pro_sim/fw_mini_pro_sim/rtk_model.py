"""WGS84 local tangent-plane RTK sensor model; driver axes are north/west/up."""
import math
import random


class RtkModel:
    def __init__(self, latitude=0., longitude=0., altitude=0., rate=10.,
                 position_std=.02, yaw_std=.005, seed=1,
                 dropout_start=-1., dropout_duration=0.):
        values = (latitude, longitude, altitude, rate, position_std, yaw_std,
                  dropout_start, dropout_duration)
        if (not all(math.isfinite(v) for v in values) or abs(latitude) >= 90 or
                abs(longitude) > 180 or rate <= 0 or position_std < 0 or
                yaw_std < 0 or dropout_duration < 0):
            raise ValueError('Invalid RTK simulation configuration')
        self.latitude, self.longitude, self.altitude = latitude, longitude, altitude
        self.rate, self.position_std, self.yaw_std = rate, position_std, yaw_std
        self.start, self.duration = dropout_start, dropout_duration
        self.random = random.Random(seed)
        self.epoch = self.last_seen = self.last_sample = None
        self.dropped = False

    def sample(self, stamp, position, yaw, forced_dropout=False):
        if not all(math.isfinite(v) for v in (stamp, *position, yaw)):
            return None
        if self.epoch is None or stamp < self.last_seen:
            self.epoch, self.last_sample = stamp, None
        self.last_seen = stamp
        elapsed = stamp - self.epoch
        self.dropped = forced_dropout or (self.start >= 0 and self.start <= elapsed < self.start + self.duration)
        if self.dropped or (self.last_sample is not None and stamp-self.last_sample < 1/self.rate-1e-9):
            return None
        self.last_sample = stamp
        return (tuple(v+self.random.gauss(0, self.position_std) for v in position),
                yaw+self.random.gauss(0, self.yaw_std))

    def geodetic(self, north, west, up):
        # LocalCartesian inverse via WGS84 ECEF, including tangent-plane curvature.
        a, e2 = 6378137., 6.6943799901413165e-3
        lat, lon = map(math.radians, (self.latitude, self.longitude))
        sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
        n = a / math.sqrt(1-e2*sl*sl)
        east = -west
        x = (n+self.altitude)*cl*co - so*east-sl*co*north+cl*co*up
        y = (n+self.altitude)*cl*so + co*east-sl*so*north+cl*so*up
        z = (n*(1-e2)+self.altitude)*sl + cl*north+sl*up
        p = math.hypot(x, y)
        phi = math.atan2(z, p*(1-e2))
        for _ in range(12):
            n = a/math.sqrt(1-e2*math.sin(phi)**2)
            phi = math.atan2(z+e2*n*math.sin(phi), p)
        height = p*math.cos(phi)+z*math.sin(phi)-a*math.sqrt(1-e2*math.sin(phi)**2)
        return math.degrees(phi), math.degrees(math.atan2(y, x)), height
