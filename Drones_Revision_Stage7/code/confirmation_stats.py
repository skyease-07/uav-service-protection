"""Prespecified mean-based scenario inference; repeated flights are not samples."""
import numpy as np
from scipy import stats


def holm(pvalues):
    order=np.argsort(pvalues); out=np.zeros(len(pvalues)); previous=0.
    for rank,index in enumerate(order):
        previous=max(previous,min(1.,(len(pvalues)-rank)*pvalues[index])); out[index]=previous
    return out.tolist()


def contrast(values,margin):
    a=np.asarray(values,float); n=len(a); mean=float(a.mean()); sd=float(a.std(ddof=1)); se=sd/np.sqrt(n)
    if se:
        t=(mean-margin)/se; p=float(stats.t.sf(t,n-1))
        ci=[mean-stats.t.ppf(.975,n-1)*se,mean+stats.t.ppf(.975,n-1)*se]
    else:
        t=None; p=0. if mean>margin else 1.; ci=[mean,mean]
    return {'n':n,'mean_pp':mean,'sd_pp':sd,'null_upper_margin_pp':margin,'t':t,'p_raw':p,
            'ci95_pp':[float(x) for x in ci],
            'family_lower_pp':float(mean-stats.t.ppf(.975,n-1)*se),
            'improved_tied_worse':[int((a>0).sum()),int((a==0).sum()),int((a<0).sum())]}


def bootstrap(values,seed=2026100856):
    a=np.asarray(values,float); rng=np.random.default_rng(seed)
    samples=a[rng.integers(0,len(a),(20000,len(a)))].mean(axis=1)
    return {'n':len(a),'mean':float(a.mean()),'ci95':np.quantile(samples,[.025,.975]).tolist()}
