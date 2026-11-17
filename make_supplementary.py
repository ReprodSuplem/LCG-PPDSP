# make_supplementary.py
# Generates the standalone IEEEtran.cls supplementary material master document (supplementary.tex)
# that compiles appendix_tables.tex into a 38-table PDF (Section VI-D).

import os

SUPPLEMENTARY_TEX = r"""\documentclass[lettersize,journal]{IEEEtran}

% Enforce IEEE RAS Double-Anonymous Rule 7: Strip PDF metadata
\pdfsuppressptexinfo=-1
\pdfinfo{
  /Author ()
  /Title ()
  /Subject ()
  /Keywords ()
  /Creator ()
  /Producer ()
}

\extrafloats{100}
\usepackage{amsmath,amsfonts,amssymb}
\usepackage{array}
\usepackage{booktabs}
\usepackage{multirow}
\usepackage{url}
\usepackage{graphicx}

% Enforce IEEE standard \footnotesize for all tables to prevent vertical page overflow
\let\oldtabular\tabular
\renewcommand{\tabular}{\footnotesize\oldtabular}

\hyphenation{op-tical net-works semi-conduc-tor IEEE-Xplore}

\begin{document}

\title{Supplementary Material: Structural Lazy Clause Generation for the Profit-Maximizing Pickup and Delivery Selection Problem}

\author{Anonymous Authors% <-this % stops a space
}

\markboth{IEEE Transactions on Automation Science and Engineering}%
{Anonymous Authors: Structural Lazy Clause Generation for the Profit-Maximizing Pickup and Delivery Selection Problem}

\maketitle

\section*{Detailed Computational Results}

Tables~\ref{tab:ppdsp_raw_A_p1} through~\ref{tab:ppdsp_raw_P_p12} report the instance-by-instance computational results for all 222 benchmark instances of the Profit-Maximizing Pickup and Delivery Selection Problem (PPDSP) derived from the CVRPLib repository across Set A, Set B, and Set P. Specifically, Tables~\ref{tab:ppdsp_raw_A_p1}--\ref{tab:ppdsp_raw_A_p14} present the 81 instances in Set A, Tables~\ref{tab:ppdsp_raw_B_p1}--\ref{tab:ppdsp_raw_B_p12} present the 69 instances in Set B, and Tables~\ref{tab:ppdsp_raw_P_p1}--\ref{tab:ppdsp_raw_P_p12} present the 72 instances in Set P. All experiments enforce a single execution thread and a wall-clock time limit of 3600 seconds per instance.

For each benchmark instance, eight exact solver configurations are evaluated: five Mixed-Integer Programming (MIP) branch-and-cut configurations, namely \textit{static}, \textit{hybrid-path}, \textit{hybrid-node}, \textit{full-path}, and \textit{full-node}; the Constraint Programming (CP) solver with Lazy Clause Generation (LCG), denoted as \textit{CP-SAT}; and two weighted partial Maximum Satisfiability (MaxSAT) LCG configurations, namely \textit{MaxSAT-path} and \textit{MaxSAT-order}. Within each instance block, winning performance is highlighted in bold: the objective value is bolded if a single configuration achieves the unique highest positive profit, whereas $T_{\text{best}}$ is bolded to indicate the fastest convergence when multiple configurations tie for the highest profit. Missing entries marked with a hyphen indicate that the configuration did not produce a valid primal solution or dual bound within the time limit.

The column headings in Tables~\ref{tab:ppdsp_raw_A_p1}--\ref{tab:ppdsp_raw_P_p12} are defined as follows:
\begin{itemize}
    \item \textbf{Instance}: The root CVRPLib instance identifier indicating the benchmark group, total number of physical locations, and fleet size.
    \item $|R|$: The total number of synthesized pickup and delivery requests in the instance pool.
    \item $k$\textbf{-NN}: The $k$-nearest neighbor connectivity parameter used to construct the sparse directed graph topology.
    \item \textbf{BKB}: The global Best Known Bound, defined as the minimum validated dual upper bound across all exact solvers or the proven optimal objective value.
    \item \textbf{Method}: The evaluated exact solver configuration.
    \item \textbf{Feas.}: A Boolean indicator set to T if the solver identifies a feasible solution and F otherwise.
    \item \textbf{TO}: A Boolean indicator set to T if the solver reaches the 3600-second time limit and F if it terminates with proven optimality.
    \item \textbf{Obj.}: The highest net profit objective value $z$ discovered by the solver within the time limit.
    \item \textbf{B.Bound}: The tightest dual upper bound validated by the individual solver at termination.
    \item \textbf{Gap(\%)}: The relative optimality gap computed against the global BKB as $(\text{BKB} - z) / z \times 100\%$, which evaluates to $\infty$ when $z = 0$.
    \item $T_{\text{best}}$\textbf{(s)}: The elapsed wall-clock time in seconds required to discover the best incumbent solution.
    \item $T_{\text{total}}$\textbf{(s)}: The total elapsed wall-clock execution time in seconds at termination.
\end{itemize}

\clearpage
__INPUT_APPENDIX_TABLES__

\end{document}
"""

def main():
    # Automatically detect whether appendix_tables.tex is in ./fiGen/ or current directory
    if os.path.exists("./fiGen/appendix_tables.tex") and not os.path.exists("appendix_tables.tex"):
        input_cmd = r"\input{./fiGen/appendix_tables.tex}"
    else:
        input_cmd = r"\input{appendix_tables.tex}"

    tex_out = SUPPLEMENTARY_TEX.replace("__INPUT_APPENDIX_TABLES__", input_cmd)
    output_path = "supplementary.tex"
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(tex_out)
    print(f"Successfully generated {output_path} (using {input_cmd}).")

if __name__ == "__main__":
    main()
