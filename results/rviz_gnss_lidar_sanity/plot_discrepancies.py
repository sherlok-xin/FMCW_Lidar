#!/usr/bin/env python3
"""Standalone descriptive plots of unchanged frozen-anchor discrepancy CSVs."""
from pathlib import Path
import csv
import json
import hashlib
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/fmcw_discrepancy_work/matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ORDER = ['100m_cross_5','100m_cross_10','200m_cross_5','200m_cross_10','300m_cross_10']
COLORS = ['#2477a8','#d78b25','#8850ad']
FIELDS = ['d_parallel_m','d_cross_horizontal_m','d_vertical_normal_m','equivalent_time_bias_s']
TITLES = ['Along-track (m)','Horizontal cross-track (m)','Vertical-normal (m)','Equivalent time bias (s)']

def rows():
    return list(csv.DictReader((HERE/'all_anchor_gnss_discrepancies.csv').open()))

def fragments(rs):
    groups=[];g=[];prev=None
    for r in rs:
        if prev and (int(r['frame_index'])!=int(prev['frame_index'])+1 or r['candidate_id']!=prev['candidate_id'] or r['gnss_segment_index']!=prev['gnss_segment_index']):
            groups.append(g);g=[]
        g.append(r);prev=r
    if g:groups.append(g)
    return groups

def main():
    data=rows();out=HERE/'figures';out.mkdir(exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
    fig,axes=plt.subplots(5,4,figsize=(14,12),layout='constrained')
    for i,cid in enumerate(ORDER):
        rs=[r for r in data if r['condition_id']==cid];t0=min(float(r['bag_record_time']) for r in rs)
        for j,(field,title) in enumerate(zip(FIELDS,TITLES)):
            ax=axes[i,j];ax.axhline(0,color='#777777',lw=.7)
            for g in fragments(rs):
                seg=int(g[0]['gnss_segment_index']);x=[float(r['bag_record_time'])-t0 for r in g];y=[float(r[field]) for r in g]
                ax.plot(x,y,'o-',ms=4,lw=1,color=COLORS[seg])
            ax.grid(alpha=.15)
            if i==0:ax.set_title(title)
            if j==0:ax.set_ylabel(cid+'\nSigned component')
            if i==4:ax.set_xlabel('Bag-record seconds since first observed anchor')
    fig.suptitle('80 observed DEV/SILVER anchors: frozen GNSS discrepancy decomposition\nNo calibration or offset fit; local-chord time equivalent is a diagnostic proxy',fontsize=13)
    handles=[plt.Line2D([0],[0],marker='o',color=c,label=f'GNSS chord {i}') for i,c in enumerate(COLORS)]
    fig.legend(handles=handles,loc='outside lower center',ncol=3,frameon=False)
    for ext in ['png','pdf']:fig.savefig(out/f'discrepancy_components_by_time.{ext}',dpi=180)
    plt.close(fig)
    cid='200m_cross_5';rs=[r for r in data if r['condition_id']==cid and 15<=int(r['frame_index'])<=21]
    r=next(r for r in rs if int(r['frame_index'])==21)
    fig,ax=plt.subplots(1,2,figsize=(12,4.7),layout='constrained')
    vals=[float(r[k]) for k in FIELDS[:3]]
    bars=ax[0].bar(['Along-track','Horizontal\ncross-track','Vertical-normal'],vals,color=['#2477a8','#d78b25','#8850ad'])
    ax[0].axhline(0,color='#777777',lw=.8);ax[0].set_ylabel('Signed component (m)')
    for b,v in zip(bars,vals):ax[0].annotate(f'{v:+.3f}',(b.get_x()+b.get_width()/2,v),ha='center',va='bottom' if v>=0 else 'top',xytext=(0,4 if v>=0 else -4),textcoords='offset points')
    ax[0].set_ylim(-6,31);ax[0].set_title(f"Frame 21: norm {float(r['discrepancy_norm_m']):.3f} m\nAlong-track squared-error share {100*float(r['parallel_squared_fraction']):.2f}%")
    x=[int(r['frame_index']) for r in rs]
    for k,label,c in zip(FIELDS[:3],['Along-track','Horizontal cross-track','Vertical-normal'],COLORS):
        ax[1].plot(x,[float(r[k]) for r in rs],'o-',color=c,label=label)
    ax[1].axhline(0,color='#777777',lw=.8);ax[1].grid(alpha=.15);ax[1].set_xlabel('Original LiDAR frame index');ax[1].set_ylabel('Signed component (m)')
    ax[1].set_title('Same observed candidate: error grows before frame 21');ax[1].legend(frameon=False)
    fig.suptitle(f"200m_cross_5 / frame21: +{float(r['equivalent_time_bias_s']):.3f} s local time equivalent\nAt final GNSS sample A70; positive shift is outside GNSS support; strict ENU-Up {float(r['d_up_enu_m']):+.3f} m",fontsize=11)
    for ext in ['png','pdf']:fig.savefig(out/f'max_frame21_components.{ext}',dpi=180)
    plt.close(fig)
    source=HERE/'all_anchor_gnss_discrepancies.csv'
    outputs=[p for p in out.glob('*.png') if p.stem in ['discrepancy_components_by_time','max_frame21_components']]+[p for p in out.glob('*.pdf')]
    manifest=dict(source=str(source.relative_to(HERE)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),matplotlib=matplotlib.__version__,
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),seed=None,randomness_introduced=False,
                  plot_definition='exact signed components; lines split across missing anchors, candidate IDs and chord boundaries; no fit',
                  output_sha256={str(p.relative_to(HERE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in outputs})
    (HERE/'decomposition_figure_manifest.json').write_text(json.dumps(manifest,indent=2))
    print('Saved component timelines and maximum-discrepancy plots')

if __name__=='__main__':main()
