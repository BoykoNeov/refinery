"""M63.2 (docs/DESIGN.md §68.2): IAPWS-IF97's saturation line at 40 digits.

The curve's slope d ln P/d(1/T) at its two ends (273.15 K, 647.096 K), which
`crates/solvers/src/thermo.rs` carries as IF97_SATURATION_END_SLOPES, and the
curve against steam tables at 100-370 °C (per cent).

Run: python -I if97_end_slopes.py   (needs mpmath)
"""
import mpmath as mp
mp.mp.dps = 40
n = [None, mp.mpf('0.11670521452767e4'), mp.mpf('-0.72421316703206e6'), mp.mpf('-0.17073846940092e2'),
     mp.mpf('0.12020824702470e5'), mp.mpf('-0.32325550322333e7'), mp.mpf('0.14915108613530e2'),
     mp.mpf('-0.48232657361591e4'), mp.mpf('0.40511340542057e6'), mp.mpf('-0.23855557567849'),
     mp.mpf('0.65017534844798e3')]
def ps(T):
    th = T + n[9]/(T - n[10])
    A = th**2 + n[1]*th + n[2]; B = n[3]*th**2 + n[4]*th + n[5]; C = n[6]*th**2 + n[7]*th + n[8]
    return (2*C/(-B + mp.sqrt(B*B - 4*A*C)))**4 * 1e6
for T in [mp.mpf('273.15'), mp.mpf('647.096')]:
    f = lambda x: mp.log(ps(1/x))
    s = mp.diff(f, 1/T)
    print(T, mp.nstr(ps(T), 20), 'dlnP/d(1/T)=', mp.nstr(s, 20))
tab = {100:1.01418,150:4.7617,200:15.549,250:39.762,300:85.879,350:165.29,370:210.43}
for t,p in tab.items(): print(t, mp.nstr(ps(mp.mpf(t)+mp.mpf('273.15'))/1e5/p*100-100, 4))
print('tb at 1 atm', mp.nstr(mp.findroot(lambda T: ps(T)-101325, 373), 12))
