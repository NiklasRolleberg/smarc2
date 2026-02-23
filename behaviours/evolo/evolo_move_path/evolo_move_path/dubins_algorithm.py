import math

# ─────────────────────────────────────────────────────────────────────────────
# Dubins Algorithm
# ─────────────────────────────────────────────────────────────────────────────

def _mod2pi(x: float) -> float:
    return x % (2.0 * math.pi)

def _dubins_LSL(alpha, beta, d):
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta),  math.cos(beta)
    tmp0 = d + sa - sb
    p_sq = 2.0 + d*d - 2.0*math.cos(alpha - beta) + 2.0*d*(sa - sb)
    if p_sq < 0: return None
    tmp1 = math.atan2(cb - ca, tmp0)
    return _mod2pi(-alpha + tmp1), math.sqrt(p_sq), _mod2pi(beta - tmp1), 'LSL'

def _dubins_RSR(alpha, beta, d):
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta),  math.cos(beta)
    tmp0 = d - sa + sb
    p_sq = 2.0 + d*d - 2.0*math.cos(alpha - beta) + 2.0*d*(sb - sa)
    if p_sq < 0: return None
    tmp1 = math.atan2(ca - cb, tmp0)
    return _mod2pi(alpha - tmp1), math.sqrt(p_sq), _mod2pi(-beta + tmp1), 'RSR'

def _dubins_LSR(alpha, beta, d):
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta),  math.cos(beta)
    p_sq = -2.0 + d*d + 2.0*math.cos(alpha - beta) + 2.0*d*(sa + sb)
    if p_sq < 0: return None
    p = math.sqrt(p_sq)
    tmp1 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return _mod2pi(-alpha + tmp1), p, _mod2pi(-_mod2pi(beta) + tmp1), 'LSR'

def _dubins_RSL(alpha, beta, d):
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta),  math.cos(beta)
    p_sq = -2.0 + d*d + 2.0*math.cos(alpha - beta) - 2.0*d*(sa + sb)
    if p_sq < 0: return None
    p = math.sqrt(p_sq)
    tmp1 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return _mod2pi(alpha - tmp1), p, _mod2pi(beta - tmp1), 'RSL'

def _dubins_RLR(alpha, beta, d):
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta),  math.cos(beta)
    tmp0 = (6.0 - d*d + 2.0*math.cos(alpha - beta) + 2.0*d*(sa - sb)) / 8.0
    if abs(tmp0) > 1.0: return None
    p = _mod2pi(2.0*math.pi - math.acos(tmp0))
    t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + _mod2pi(p / 2.0))
    return t, p, _mod2pi(alpha - beta - t + _mod2pi(p)), 'RLR'

def _dubins_LRL(alpha, beta, d):
    sa, ca = math.sin(alpha), math.cos(alpha)
    sb, cb = math.sin(beta),  math.cos(beta)
    tmp0 = (6.0 - d*d + 2.0*math.cos(alpha - beta) + 2.0*d*(-sa + sb)) / 8.0
    if abs(tmp0) > 1.0: return None
    p = _mod2pi(2.0*math.pi - math.acos(tmp0))
    t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    return t, p, _mod2pi(_mod2pi(beta) - alpha - t + _mod2pi(p)), 'LRL'

def _sample_segment(x0, y0, yaw0, seg_type, length, radius, step):
    pts, dist, x, y, yaw = [], 0.0, x0, y0, yaw0
    while dist < length * radius:
        pts.append((x, y, yaw))
        if seg_type == 'S':
            x += step * math.cos(yaw); y += step * math.sin(yaw)
        elif seg_type == 'L':
            dy = step / radius
            x += radius * (math.sin(yaw + dy) - math.sin(yaw))
            y += radius * (-math.cos(yaw + dy) + math.cos(yaw))
            yaw += dy
        elif seg_type == 'R':
            dy = step / radius
            x += radius * (-math.sin(yaw - dy) + math.sin(yaw))
            y += radius * (math.cos(yaw - dy) - math.cos(yaw))
            yaw -= dy
        dist += step
    return pts

def dubins_path(q0, q1, radius, step):
    x0, y0, yaw0 = q0
    x1, y1, yaw1 = q1
    dx, dy = x1 - x0, y1 - y0
    D = math.hypot(dx, dy)
    d = D / radius
    theta = _mod2pi(math.atan2(dy, dx))
    alpha = _mod2pi(yaw0 - theta)
    beta  = _mod2pi(yaw1 - theta)

    best, best_len = None, float('inf')
    for fn in [_dubins_LSL, _dubins_RSR, _dubins_LSR,
               _dubins_RSL, _dubins_RLR, _dubins_LRL]:
        c = fn(alpha, beta, d)
        if c is None: continue
        t, p, q, mode = c
        if t + p + q < best_len:
            best_len = t + p + q
            best = c

    if best is None:
        n = max(2, int(D / step))
        return [(x0 + i/n*dx, y0 + i/n*dy, math.atan2(dy, dx)) for i in range(n+1)]

    t, p, q, mode = best
    path_pts, cx, cy, cyaw = [], x0, y0, yaw0
    for seg_type, length in zip(list(mode), [t, p, q]):
        if length < 1e-6: continue
        seg = _sample_segment(cx, cy, cyaw, seg_type, length, radius, step)
        if seg:
            path_pts.extend(seg)
            cx, cy, cyaw = seg[-1]
    path_pts.append((x1, y1, yaw1))
    return path_pts

