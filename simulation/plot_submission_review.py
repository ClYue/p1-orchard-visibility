"""Render the November 6 review figures without replacing submission files."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Ellipse, Rectangle
from matplotlib.ticker import FixedLocator, FuncFormatter, NullFormatter
import numpy as np


ROOT = Path(__file__).resolve().parent
INK, MUTED, GRID = "#162838", "#506477", "#DCE4EB"
BLUE, AMBER, PURPLE, TEAL = "#146B9C", "#AE5415", "#7655A2", "#138879"
SCENES = {"center": ("Alley center", BLUE, "o"), "edge": ("Canopy edge", AMBER, "s")}
PRIORS = {"lut": ("Lookup table", BLUE, "o"),
          "gaussian": ("Centered Gaussian", AMBER, "^"),
          "uniform": ("Uniform", PURPLE, "D")}


def load_study(directory):
    return json.loads((directory / "study.json").read_text(encoding="utf-8"))


def save(fig, output, name):
    fig.savefig(output / f"{name}.png", dpi=600)
    fig.savefig(output / f"{name}.pdf")
    plt.close(fig)


def panel(ax, letter, title=""):
    ax.set_title(f"({letter})   {title}".rstrip(), loc="left", pad=7)


def style_axis(ax, ticks=None):
    ax.grid(axis="y", color=GRID, linewidth=.6)
    ax.set_axisbelow(True)
    if ticks is not None:
        ax.set_yscale("log")
        ax.yaxis.set_major_locator(FixedLocator(ticks))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda value, _: f"{value:g}"))
        ax.yaxis.set_minor_formatter(NullFormatter())
        ax.tick_params(axis="y", which="minor", length=0)


def bootstrap(samples):
    n = samples.shape[-1]
    draws = np.random.default_rng(91).integers(n, size=(5000, n))
    mean = samples.mean(axis=-1)
    lo, hi = np.quantile(samples[..., draws].mean(axis=-1), [.025, .975], axis=-1)
    return mean, np.maximum(0, [mean-lo, hi-mean])


def best_fixed(rows):
    return min((r for r in rows if r["model"] == "fixed"), key=lambda r: r["mean"][1])


def chosen_lut(rows):
    return next(r for r in rows if r["model"] == "lut" and r["weight"] == .001)


def visibility(study, inputs, output):
    fig, axs = plt.subplots(2, 2, figsize=(6.55, 4.9), sharex=True, sharey=True)
    fig.subplots_adjust(left=.12, right=.86, bottom=.105, top=.89, wspace=.085, hspace=.17)
    fig.text(.302, .965, "Ray-based lookup table", ha="center", fontsize=9.4, weight="bold")
    fig.text(.68, .965, "Bounded Gaussian fit", ha="center", fontsize=9.4, weight="bold")
    with np.load(inputs / "priors.npz") as maps:
        g = maps["grid"]
        x, y = np.meshgrid(g, g)
        zero = np.argmin(abs(g))
        for i, (key, (scene, _, _)) in enumerate(SCENES.items()):
            fit = study["scenes"][key]
            a, sx, sy = fit["fit"]
            surfaces = [maps[key], a * np.exp(-(x/sx)**2-(y/sy)**2)]
            yband = float(g[np.argmax(maps[key][:, zero])]) if i == 1 else None
            for j, surface in enumerate(surfaces):
                ax = axs[i, j]
                heat = ax.pcolormesh(g, g, surface, cmap="viridis", vmin=0, vmax=1,
                                     shading="nearest", rasterized=False)
                ax.contour(x, y, surface, [.5], colors="white", linewidths=.8,
                           linestyles="dashed")
                ax.scatter([0], [0], marker="+", s=38, linewidths=1.1, c="white")
                ax.set(xlim=(-15, 15), ylim=(-12, 12), aspect="equal",
                       xticks=[-10, 0, 10], yticks=[-10, 0, 10])
                ax.text(.035, .93, f"({'abcd'[2*i+j]})  {scene}", transform=ax.transAxes,
                        color="white", fontsize=8, weight="semibold", va="top")
                text = (rf"$p_0={surface[zero,zero]:.3f}$" if j == 0
                        else rf"$R^2={fit['r2']:.3f}$")
                ax.text(.035, .057, text, transform=ax.transAxes, color="white",
                        fontsize=8.5, va="bottom")
                if i == 1:
                    if j == 0:
                        ax.annotate("", xy=(0, yband), xytext=(0, 0),
                                    arrowprops={"arrowstyle": "<->", "color": "white", "lw": 1.1})
                        ax.text(1.4, yband*.5, f"{yband:+.1f} m", color=INK, fontsize=7.4,
                                va="center", ha="left", weight="semibold")
                    else:
                        ax.axhline(yband, color="white", lw=.85,
                                   ls=(0, (3, 3)), alpha=.95)
                        ax.text(-14.2, yband-1.1, "where the band sits", color="white",
                                fontsize=7.0, ha="left", va="top")
        for ax in axs[:, 0]:
            ax.set_ylabel("Across-row offset (m)")
        fig.supxlabel("Along-row offset (m)", x=.485, y=.025)
        cax = fig.add_axes([.881, .20, .021, .62])
        cb = fig.colorbar(heat, cax=cax, ticks=np.linspace(0, 1, 6))
        cb.set_label("Pair visibility before camera gate", labelpad=6)
        cb.outline.set_linewidth(.55)
    save(fig, output, "Figure1")


def tradeoffs(study, output):
    fig, axs = plt.subplots(2, 2, figsize=(6.55, 5.24), sharex=True, sharey="col")
    fig.subplots_adjust(left=.12, right=.978, bottom=.095, top=.82, wspace=.25, hspace=.33)
    for i, (key, (scene, _, _)) in enumerate(SCENES.items()):
        rows = study["scenes"][key]["results"]
        fixed = np.array([r["mean"] for r in rows if r["model"] == "fixed"])
        base, lut = best_fixed(rows)["mean"], chosen_lut(rows)["mean"]
        for j, (ax, idx, label) in enumerate(zip(axs[i], [1, 0], ["Heading RMSE (°)", "Position RMSE (m)"])):
            panel(ax, "abcd"[2*i+j], scene)
            ax.scatter(fixed[:, 2], fixed[:, idx], s=16, c="#D7E0E7",
                       edgecolors="#7A8C9A", lw=.6, zorder=2)
            for key2, (_, c, marker) in PRIORS.items():
                pts = np.array([r["mean"] for r in rows if r["model"] == key2])
                ax.scatter(pts[:, 2], pts[:, idx], color=c, marker=marker,
                           s=27, edgecolor="white", lw=.35, zorder=4)
            ax.scatter(base[2], base[idx], marker="*", s=98, c=INK, edgecolor="white", lw=.5, zorder=6)
            ax.scatter(lut[2], lut[idx], marker="o", s=70, facecolor="none",
                       edgecolor=BLUE, lw=1.05, zorder=5)
            if i == 1 and j == 0:
                ax.annotate("26.4 m / 0.749°", (lut[2], lut[idx]), xytext=(60, .87),
                            color=BLUE, fontsize=7.3,
                            arrowprops={"arrowstyle": "-", "color": BLUE, "lw": .7})
                ax.annotate("31.2 m / 0.562°", (base[2], base[idx]), xytext=(60, .39),
                            color=INK, fontsize=7.3,
                            arrowprops={"arrowstyle": "-", "color": MUTED, "lw": .7})
            style_axis(ax, [.3,.5,1,2,5,10,20] if j == 0 else [.01,.03,.1,.3,1,3])
            ax.set(ylabel=label, xlim=(-3, 115), xticks=np.arange(0, 101, 20))
    fig.supxlabel("Cumulative UAV flight distance (m)", y=.024)
    legend = [
        Line2D([], [], ls="", marker="o", mfc="#D7E0E7", mec="#7A8C9A", ms=4.4, label="Fixed station"),
        Line2D([], [], ls="", marker="*", color=INK, ms=8, label="Best-heading fixed"),
        *(Line2D([], [], ls="", marker=marker, color=c, ms=5, label=label)
          for label, c, marker in PRIORS.values()),
        Line2D([], [], ls="", marker="o", mfc="none", mec=BLUE, ms=6.5, label=r"Lookup table, $w=0.001$"),
    ]
    fig.legend(handles=legend, loc="upper center", bbox_to_anchor=(.53,.968),
               ncol=3, frameon=False, fontsize=8.0, columnspacing=1.3)
    save(fig, output, "Figure2")


def azimuth(study, output):
    fig, axs = plt.subplots(2, 1, figsize=(6.55, 4.35), sharex=True)
    fig.subplots_adjust(left=.12, right=.972, bottom=.185, top=.85, hspace=.34)
    for key, (label, c, marker) in SCENES.items():
        rows = sorted((r for r in study["scenes"][key]["results"]
                       if r["model"] == "fixed" and r["radius"] == 2),
                      key=lambda r: r["angle"])
        angles = np.array([r["angle"] for r in rows])
        samples = np.array([r["samples"] for r in rows])
        for ax, idx, factor in zip(axs, (1,3), (1,1)):
            mean, yerr = bootstrap(samples[:,:,idx]*factor)
            ax.errorbar(angles, mean, yerr=yerr, color=c, marker=marker,
                        ls="-" if key == "center" else "--", label=label,
                        ms=5.1, lw=1.2, elinewidth=.85, capsize=2.5, zorder=3,
                        capthick=.85, alpha=.95)
    for ax in axs:
        ax.axvspan(30, 150, color="#F8EFE7", lw=0, zorder=0)
    panel(axs[0], "a", "Heading error")
    panel(axs[1], "b", "Paired marker detections")
    style_axis(axs[0], [.3,.5,1,2,5,10])
    style_axis(axs[1])
    axs[0].set(ylabel="Heading RMSE (°)", ylim=(.27,18))
    axs[1].set(ylabel="Pair detection fraction", ylim=(-.04,1.08), yticks=[0,.25,.5,.75,1],
               xlim=(-11,326), xlabel="Requested station azimuth in the vehicle frame")
    edge = np.array([r["mean"][1] for r in sorted(
        (r for r in study["scenes"]["edge"]["results"]
         if r["model"] == "fixed" and r["radius"] == 2), key=lambda r: r["angle"])])
    lo, hi = float(edge.min()), float(edge.max())
    axs[0].axhline(lo, color=AMBER, lw=.6, ls=(0, (2, 3)), alpha=.55, zorder=1)
    axs[0].annotate("", xy=(303, hi), xytext=(303, lo),
                    arrowprops={"arrowstyle": "<->", "color": AMBER, "lw": .9})
    axs[0].text(291, np.sqrt(lo*hi), f"{hi/lo:.1f}×", color=AMBER, fontsize=7.6,
                ha="right", va="center")
    names = {0:"Forward",90:"Left",180:"Rear",270:"Right"}
    angles = np.arange(0,360,45)
    axs[1].set_xticks(angles, [f"{a}°"+("\n"+names[a] if a in names else "") for a in angles])
    fig.legend(*axs[0].get_legend_handles_labels(), loc="upper center",
               bbox_to_anchor=(.54,.975), ncol=2, frameon=False)
    save(fig, output, "Figure3")


def crowns(study, inputs, output):
    widths = [1.2,1.5,1.8,2.0]
    pairs = []
    for width in widths:
        dataset = study if width == 1.2 else load_study(inputs.parent/f"results_crown_b{width:.1f}")
        rows = dataset["scenes"]["edge"]["results"]
        pairs.append([best_fixed(rows)["samples"], chosen_lut(rows)["samples"]])
    pairs = np.array(pairs)
    fig, axs = plt.subplots(1,2, figsize=(6.55,3.1), sharex=True)
    fig.subplots_adjust(left=.115, right=.969, bottom=.17, top=.75, wspace=.31)
    for j, (ax,idx,title,unit) in enumerate(zip(
        axs, (1,2), ("Heading error","Flight distance"), ("Heading RMSE (°)","UAV flight distance (m)")
    )):
        for k, (label,c,marker,ls) in enumerate([
            ("Best-heading fixed",INK,"*","--"),(r"Lookup table, $w=0.001$",BLUE,"o","-")
        ]):
            means, err = bootstrap(pairs[:,k,:,idx])
            ax.errorbar(widths, means, yerr=err, color=c, marker=marker, ls=ls,
                        ms=6.3 if k==0 else 4.8, lw=1.25, elinewidth=.8, capsize=2.6,
                        label=label)
        panel(ax, "ab"[j], title)
        style_axis(ax)
        ax.set(ylabel=unit, xticks=widths, xlim=(1.11,2.09), ylim=(0,None))
    fig.supxlabel("Across-row crown semi-axis (m)", y=.026)
    fig.legend(*axs[0].get_legend_handles_labels(), loc="upper center",
               bbox_to_anchor=(.53,.963), ncol=2, frameon=False)
    save(fig, output, "Figure4")


def scheme(output):
    """A conceptual diagram of the nominal configuration and the ray model."""
    fig, (a,b) = plt.subplots(1,2,figsize=(6.55,3.2),gridspec_kw={"width_ratios":[1.12,1]})
    fig.subplots_adjust(left=.08,right=.985,bottom=.145,top=.85,wspace=.30)
    crown = "#ACD0AF"
    for row in (-2,2):
        for x in np.arange(-4,5,2):
            a.add_patch(Ellipse((x,row),2.4,2.4,fc=crown,ec="#689779",lw=.65,alpha=.76))
    a.axhline(0,color=MUTED,lw=.65,ls="--",alpha=.75)
    for yy, color in [(0,BLUE),(1,AMBER)]:
        a.plot([-3.7,3.0],[yy,yy],color=color,lw=1.1,alpha=.8)
        a.add_patch(Rectangle((-.62,yy-.16),1.24,.32,fc="white",ec=color,lw=.9,zorder=4))
        a.scatter([-.5,.5],[yy,yy],s=17,c=color,zorder=5)
        a.annotate("",(3.45,yy),(2.93,yy),arrowprops={"arrowstyle":"->","color":color,"lw":1.0})
    a.text(-4.8,-.42,"Center path",color=BLUE,fontsize=7.4)
    a.text(-4.8,.56,"Offset path",color=AMBER,fontsize=7.4)
    a.annotate("",(-.5,-.63),(.5,-.63),arrowprops={"arrowstyle":"<->","color":MUTED,"lw":.7})
    a.text(0,-1.02,"1 m marker baseline",ha="center",color=MUTED,fontsize=7.3)
    a.annotate("",(4.48,-2),(4.48,2),arrowprops={"arrowstyle":"<->","color":INK,"lw":.75})
    a.text(4.61,.48,"4 m",rotation=90,va="center",fontsize=7.5)
    a.set(xlim=(-5,5.15),ylim=(-3.45,3.45),aspect="equal",
          xlabel="Along-row direction",ylabel="Across-row direction",xticks=[],yticks=[])
    panel(a,"a","Orchard plan view")
    for xx in (-2,2):
        b.add_patch(Ellipse((xx,2),2.4,3,fc=crown,ec="#689779",lw=.8,alpha=.8))
    target=(1,.3)
    blocked="#A8201A"
    for xx,color,label in [(-2.8,BLUE,"Far-side view"),(3.25,AMBER,"Near-side view")]:
        b.plot([xx,target[0]],[6,target[1]],color=color,lw=1.2,
               ls="-" if color==BLUE else "--",alpha=.85,zorder=3)
        b.scatter([xx],[6],s=34,marker="v",c=color,zorder=5)
        b.text(xx,6.24,label,color=color,ha="center",fontsize=7.4)
        # Portion of the ray that lies inside a crown, drawn on top so the
        # interception is visible rather than implied.
        ts=np.linspace(0,1,601)
        pts=np.array([xx,6.])[None,:]+ts[:,None]*(np.array(target)-np.array([xx,6.]))[None,:]
        inside=np.zeros(ts.size,bool)
        for cx in (-2,2):
            inside |= ((pts[:,0]-cx)/1.2)**2+((pts[:,1]-2)/1.5)**2<=1.0
        hit=np.flatnonzero(inside)
        if hit.size:
            for run in np.split(hit,np.flatnonzero(np.diff(hit)>1)+1):
                b.plot(pts[run,0],pts[run,1],color=blocked,lw=2.4,
                       solid_capstyle="round",zorder=6)
    b.annotate("intercepted\nby canopy",(1.95,2.45),xytext=(.15,4.35),color=blocked,
               fontsize=7.2,ha="center",va="center",
               arrowprops={"arrowstyle":"-","color":blocked,"lw":.7})
    b.scatter([target[0]],[target[1]],s=24,marker="s",c=INK,zorder=5)
    b.annotate("Marker",target,(.85,.85),fontsize=7.5,
               arrowprops={"arrowstyle":"-","color":MUTED,"lw":.7})
    b.set(xlim=(-4.5,4.65),ylim=(0,6.75),xlabel="Across-row offset (m)",
          ylabel="Height (m)",yticks=[0,2,4,6],xticks=[-4,-2,0,2,4])
    panel(b,"b","Ray paths across the alley")
    save(fig,output,"Optional_Setup_Schematic")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,default=ROOT/"results")
    parser.add_argument("--output",type=Path,default=ROOT/"review_20261106"/"figures")
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({
        "font.family":"DejaVu Sans","font.size":8.5,"axes.labelsize":9,
        "axes.titlesize":9.1,"axes.titleweight":"bold","axes.labelcolor":INK,
        "text.color":INK,"xtick.color":INK,"ytick.color":INK,"xtick.labelsize":8,
        "ytick.labelsize":8,"axes.edgecolor":MUTED,"axes.linewidth":.65,
        "axes.spines.top":False,"axes.spines.right":False,
        "xtick.major.size":3,"ytick.major.size":3,"pdf.fonttype":42,
    })
    study=load_study(args.input)
    visibility(study,args.input,args.output)
    tradeoffs(study,args.output)
    azimuth(study,args.output)
    crowns(study,args.input,args.output)
    scheme(args.output)


if __name__=="__main__":
    main()
