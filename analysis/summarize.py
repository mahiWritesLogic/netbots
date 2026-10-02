"""Print mean±std per variant for every experiment under results/.

    python analysis/summarize.py [results_dir]
"""
import csv
import os
import statistics as st
import sys

COLS = ["pdr", "lat_mean_ms", "lat_p95_ms", "aoi_mean_s", "overhead_ip_Bps", "drop_rate_queue",
        "msg_out_of_order", "gap_err_mean_m", "gap_err_excess_m", "dev_from_ref_mean_m"]


def main(root):
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, "summary.csv")
        if not os.path.exists(path):
            continue
        rows = list(csv.DictReader(open(path)))
        print(f"\n## {name}  (n_seeds={len({r['seed'] for r in rows})})\n")
        print("| variant | " + " | ".join(COLS) + " |")
        print("|---" * (len(COLS) + 1) + "|")
        for lab in dict.fromkeys(r["label"] for r in rows):
            cells = []
            for c in COLS:
                v = [float(r[c]) for r in rows if r["label"] == lab and r.get(c) not in (None, "")]
                cells.append(f"{st.mean(v):.3g} ± {st.stdev(v) if len(v) > 1 else 0:.2g}" if v else "")
            print(f"| {lab} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results"))
