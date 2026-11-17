# make_latex_tables.py
# Post-processing script that reads ppdsp_results.csv to generate the solution quality
# dominance summary table (section6_table.tex, Table II) and the 38-table instance-by-instance
# supplementary document body (appendix_tables.tex) (Section VI-D).

import pandas as pd
import numpy as np

METHODS = [
    'static',
    'hybrid_path', 'hybrid_node',
    'full_path', 'full_node',
    'cpsat',
    'maxsat_path', 'maxsat_order'
]

METHOD_LATEX_MAP = {
    'static': '\\textit{static}',
    'hybrid_path': '\\textit{hybrid-path}',
    'hybrid_node': '\\textit{hybrid-node}',
    'full_path': '\\textit{full-path}',
    'full_node': '\\textit{full-node}',
    'cpsat': '\\textit{CP-SAT}',
    'maxsat_path': '\\textit{MaxSAT-path}',
    'maxsat_order': '\\textit{MaxSAT-order}'
}

def generate_vertical_latex(df, base_caption, label_prefix, chunk_size=6, is_maximization=True):
    latex_output = ""
    
    # Sort data to maintain a consistent hierarchical order
    df = df.sort_values(by=['Instance', 'Requests', 'K', 'Method'])
    unique_insts = df[['Instance', 'Requests', 'K', 'BKB']].drop_duplicates()
    
    total_chunks = int(np.ceil(len(unique_insts) / chunk_size))
    
    for chunk_idx in range(total_chunks):
        chunk_insts = unique_insts.iloc[chunk_idx * chunk_size : (chunk_idx + 1) * chunk_size]
        
        # Handle multi-part table captions and labels dynamically
        if total_chunks > 1:
            caption = f"{base_caption} Part {chunk_idx + 1}."
            label = f"{label_prefix}_p{chunk_idx + 1}"
        else:
            caption = f"{base_caption}"
            label = label_prefix
            
        latex = f"\\begin{{table*}}[t]\n\\centering\n"
        latex += f"\\caption{{{caption}}}\n"
        latex += f"\\label{{{label}}}\n"
        latex += "\\begin{tabular}{lllr l ccrrrrr}\n\\toprule\n"
        latex += "Instance & $|R|$ & $k$-NN & BKB & Method & Feas. & TO & Obj. & B.Bound & Gap(\\%) & $T_{\\text{best}}$(s) & $T_{\\text{total}}$(s) \\\\\n\\midrule\n"
        
        for _, group in chunk_insts.iterrows():
            inst = group['Instance']
            req = group['Requests']
            k = group['K']
            bkb = group['BKB']
            
            escaped_inst = inst.replace("_", "\\_")
            safe_inst = f"\\texttt{{{escaped_inst}}}"
            bkb_str = f"{bkb:.0f}" if pd.notna(bkb) else "-"
            
            sub_df = df[(df['Instance']==inst) & (df['Requests']==req) & (df['K']==k)].copy()
            
            # Enforce strict solver display order
            sub_df['Method_Cat'] = pd.Categorical(sub_df['Method'], categories=METHODS, ordered=True)
            sub_df = sub_df.sort_values('Method_Cat')
            
            # Determine winning criteria based on the optimization direction
            valid_objs = sub_df[(sub_df['Objective'].notna()) & (sub_df['Objective'] > 0)]['Objective']
            best_obj = None
            if not valid_objs.empty:
                best_obj = valid_objs.max() if is_maximization else valid_objs.min()
            
            has_tie = False
            tie_min_tbest = None
            if best_obj is not None:
                best_rows = sub_df[abs(sub_df['Objective'] - best_obj) < 1e-5]
                if len(best_rows) > 1:
                    has_tie = True
                    valid_tbest = best_rows['Time_to_Best(s)'].dropna()
                    if not valid_tbest.empty:
                        tie_min_tbest = valid_tbest.min()
            
            first_row = True
            for _, row in sub_df.iterrows():
                m_name = METHOD_LATEX_MAP.get(row['Method'], row['Method'])
                
                # Format boolean indicators
                feas = "T" if row['HasFeasible'] else "F"
                to = "T" if row['Timeout'] else "F"
                
                # Sanitize floating-point artifacts
                obj_val = row['Objective']
                t_best_val = row['Time_to_Best(s)']
                
                is_obj_best = False
                if pd.notna(obj_val) and best_obj is not None and abs(obj_val - best_obj) < 1e-5:
                    is_obj_best = True
                
                # Format Objective: Bold only if it is the unique highest objective value
                if pd.notna(obj_val):
                    if obj_val == 0 or obj_val == -0.0:
                        obj = "0"
                    else:
                        obj_base = f"{obj_val:.0f}"
                        if is_obj_best and not has_tie:
                            obj = f"\\textbf{{{obj_base}}}"
                        else:
                            obj = obj_base
                else:
                    obj = "-"
                    
                # Format Time_to_Best: Bold only if it resolves a tie in best objective
                if pd.notna(t_best_val):
                    t_best_base = f"{t_best_val:.2f}"
                    if has_tie and is_obj_best and tie_min_tbest is not None and abs(t_best_val - tie_min_tbest) < 1e-3:
                        t_best = f"\\textbf{{{t_best_base}}}"
                    else:
                        t_best = t_best_base
                else:
                    t_best = "-"
                
                # Render mathematically correct infinity symbols for Bounds and Gaps
                if pd.notna(row['BestBound']):
                    bb = "$\\infty$" if np.isinf(row['BestBound']) else f"{row['BestBound']:.0f}"
                else:
                    bb = "-"
                    
                if pd.notna(row['Gap(%)']):
                    gap = "$\\infty$" if np.isinf(row['Gap(%)']) else f"{row['Gap(%)']:.2f}"
                else:
                    gap = "-"

                t_tot_val = row['Total_Time(s)']
                t_tot = f"{t_tot_val:.2f}" if pd.notna(t_tot_val) else "-"
                
                # Clear irrelevant outputs for infeasible runs
                if row['HasFeasible'] == False:
                    obj = "-"
                    gap = "-"
                
                if first_row:
                    latex += f"{safe_inst} & {req} & {k} & {bkb_str} & {m_name} & {feas} & {to} & {obj} & {bb} & {gap} & {t_best} & {t_tot} \\\\\n"
                    first_row = False
                else:
                    latex += f" &  &  &  & {m_name} & {feas} & {to} & {obj} & {bb} & {gap} & {t_best} & {t_tot} \\\\\n"
                    
            latex += "\\midrule\n"
            
        latex = latex.rsplit("\\midrule\n", 1)[0] + "\\bottomrule\n\\end{tabular}\n\\end{table*}\n\\clearpage\n\n"
        latex_output += latex
        
    return latex_output

def calculate_wins(df, is_maximization=True):
    """
    Evaluate algorithmic dominance using the same logic applied in the detail tables.
    Winner logic: Best positive objective value, with ties broken by shortest Time_to_Best(s).
    """
    counts = {m: 0 for m in METHODS}
    
    for (inst, req, k), inst_df in df.groupby(['Instance', 'Requests', 'K']):
        valid_rows = inst_df[(inst_df['Objective'].notna()) & (inst_df['Objective'] > 0)].copy()
        if valid_rows.empty:
            continue
            
        valid_rows['Method_Cat'] = pd.Categorical(valid_rows['Method'], categories=METHODS, ordered=True)
        valid_rows = valid_rows.sort_values('Method_Cat')

        best_obj = valid_rows['Objective'].max() if is_maximization else valid_rows['Objective'].min()
        best_rows = valid_rows[abs(valid_rows['Objective'] - best_obj) < 1e-5]
        
        if len(best_rows) == 1:
            winner_method = best_rows.iloc[0]['Method']
        else:
            valid_tbest = best_rows['Time_to_Best(s)'].dropna()
            if not valid_tbest.empty:
                tie_min_tbest = valid_tbest.min()
                winner_candidates = best_rows[abs(best_rows['Time_to_Best(s)'] - tie_min_tbest) < 1e-3]
                winner_method = winner_candidates.iloc[0]['Method']
            else:
                winner_method = best_rows.iloc[0]['Method']
            
        if winner_method in counts:
            counts[winner_method] += 1
            
    return counts

def generate_summary_table(ppdsp_df):
    """
    Generate the Solution Quality Dominance summary table (Table II in Section VI-D).
    """
    ppdsp_df = ppdsp_df.copy()
    def get_ppdsp_group(name):
        if name.startswith('A-'): return 'Set A'
        elif name.startswith('B-'): return 'Set B'
        elif name.startswith('P-'): return 'Set P'
        return 'other'
        
    ppdsp_df['SummaryGroup'] = ppdsp_df['Instance'].apply(get_ppdsp_group)
    
    groups = ['Set A', 'Set B', 'Set P']
    results = {}
    for grp in groups:
        grp_df = ppdsp_df[ppdsp_df['SummaryGroup'] == grp]
        results[grp] = calculate_wins(grp_df, is_maximization=True)
    results['Total'] = calculate_wins(ppdsp_df, is_maximization=True)
        
    latex = "\\begin{table*}[!t]\n\\centering\n"
    latex += "\\caption{Solution Quality Dominance Across the PPDSP Benchmark Sets. Values Indicate the Total Number of Instances Where Each Solver Configuration Achieved the Best Objective Value.}\n"
    latex += "\\label{tab:dominance_results}\n"
    latex += "\\small\n\\setlength{\\tabcolsep}{2.2mm}\n"
    latex += "\\begin{tabular}{@{}lcccccccc@{}}\n\\toprule\n"
    latex += "\\textbf{Group} & \\textit{static} & \\textit{hybrid-path} & \\textit{hybrid-node} & \\textit{full-path} & \\textit{full-node} & \\textit{CP-SAT} & \\textit{MaxSAT-path} & \\textit{MaxSAT-order} \\\\ \\midrule\n"
    
    for grp in ['Set A', 'Set B', 'Set P', 'Total']:
        if grp == 'Total':
            latex += "\\midrule\n"
        r = results[grp]
        max_wins = max(r.values()) if r.values() else 0
        row_strs = []
        for m in METHODS:
            val = r[m]
            if val == max_wins and val > 0:
                row_strs.append(f"\\textbf{{{val}}}")
            else:
                row_strs.append(str(val))
        grp_label = "\\textbf{Total}" if grp == 'Total' else grp
        latex += f"{grp_label} & {' & '.join(row_strs)} \\\\\n"
        
    latex += "\\bottomrule\n\\end{tabular}\n\\end{table*}\n"
    
    return latex

def main():
    df_ppdsp = pd.read_csv('ppdsp_results.csv')

    cap_base = "Detailed computational results for the {name} instances. Feas. indicates if a feasible solution was found. TO indicates Timeout (3600s). BKB denotes the global best known bound. B.Bound indicates the best dual bound discovered by the solver. $T_{{\\text{{best}}}}$ and $T_{{\\text{{total}}}}$ represent the time in seconds to find the best incumbent and the total execution time, respectively. Winning criteria are highlighted in bold: the objective value is bolded if uniquely optimal, or $T_{{\\text{{best}}}}$ is bolded if it breaks a tie among equivalent objectives."

    df_ppdsp_A = df_ppdsp[df_ppdsp['Instance'].str.startswith('A-')]
    latex_ppdsp_A = generate_vertical_latex(df_ppdsp_A, cap_base.format(name="PPDSP Set A"), "tab:ppdsp_raw_A", chunk_size=6, is_maximization=True)
    
    df_ppdsp_B = df_ppdsp[df_ppdsp['Instance'].str.startswith('B-')]
    latex_ppdsp_B = generate_vertical_latex(df_ppdsp_B, cap_base.format(name="PPDSP Set B"), "tab:ppdsp_raw_B", chunk_size=6, is_maximization=True)
    
    df_ppdsp_P = df_ppdsp[df_ppdsp['Instance'].str.startswith('P-')]
    latex_ppdsp_P = generate_vertical_latex(df_ppdsp_P, cap_base.format(name="PPDSP Set P"), "tab:ppdsp_raw_P", chunk_size=6, is_maximization=True)

    with open('appendix_tables.tex', 'w', encoding='utf-8') as f:
        f.write("% ---- Append to your LaTeX file ----\n\n")
        f.write(latex_ppdsp_A + latex_ppdsp_B + latex_ppdsp_P)
        
    latex_summary = generate_summary_table(df_ppdsp)
    
    with open('section6_table.tex', 'w', encoding='utf-8') as f:
        f.write("% ---- Insert into Section 6 via \\input{section6_table.tex} ----\n\n")
        f.write(latex_summary)
    
    print("Generation complete.")
    print("-> Created appendix_tables.tex")
    print("-> Created section6_table.tex")

if __name__ == "__main__":
    main()
