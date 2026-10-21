import pandas as pd
import numpy as np
import os

METHODS = [
    'static',
    'hybrid_path', 'hybrid_node',
    'full_path', 'full_node',
    'cpsat',
    'maxsat_path', 'maxsat_order'
]

def compute_coordinates(df_group):
    out_data = {}
    for m in METHODS:
        m_df = df_group[df_group['Method'] == m]
        valid_rows = m_df[m_df['HasFeasible'] == True]
        
        # Ensure values are parsed as floats to correctly handle 'inf' strings
        gaps = valid_rows['Gap(%)'].astype(float).dropna().values
        
        if len(gaps) > 0:
            # INTERCEPT INF: Replace np.inf with 100000.0 to prevent LaTeX compilation crash
            gaps = np.array([100000.0 if np.isinf(g) else max(g, 0.1) for g in gaps])
            gaps = np.sort(gaps)
            y_vals = np.arange(1, len(gaps) + 1)
            
            x_coords = np.concatenate(([0.1], gaps, [100000.0]))
            y_coords = np.concatenate(([0], y_vals, [len(gaps)]))
            
            out_data[f'{m}_x'] = x_coords
            out_data[f'{m}_y'] = y_coords
        else:
            out_data[f'{m}_x'] = [0.1, 100000.0]
            out_data[f'{m}_y'] = [0, 0]
            
    max_len = max([len(v) for v in out_data.values()] + [0])
    for k in out_data:
        out_data[k] = np.append(out_data[k], [np.nan] * (max_len - len(out_data[k])))
        
    return pd.DataFrame(out_data)

if __name__ == "__main__":
    if os.path.exists('ppdsp_results.csv'):
        ppdsp_df = pd.read_csv('ppdsp_results.csv')
        def get_ppdsp_group(name):
            if name.startswith('A-'): return 'Set A'
            elif name.startswith('B-'): return 'Set B'
            elif name.startswith('P-'): return 'Set P'
            return 'unknown'
        ppdsp_df['Group'] = ppdsp_df['Instance'].apply(get_ppdsp_group)
        
        # Individual groups
        compute_coordinates(ppdsp_df[ppdsp_df['Group'] == 'Set A']).to_csv('ppdsp_A_gap.csv', index=False)
        compute_coordinates(ppdsp_df[ppdsp_df['Group'] == 'Set B']).to_csv('ppdsp_B_gap.csv', index=False)
        compute_coordinates(ppdsp_df[ppdsp_df['Group'] == 'Set P']).to_csv('ppdsp_P_gap.csv', index=False)
        # Aggregated total group
        compute_coordinates(ppdsp_df).to_csv('ppdsp_all_gap.csv', index=False)
        print("PPDSP processing complete.")
    else:
        print("[Error] ppdsp_results.csv not found.")
