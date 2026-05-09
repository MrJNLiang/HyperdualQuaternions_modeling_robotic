import os
import sys
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.run_disturbance import run_one_case, summarize_runtime


def main():
    methods = ["numeric", "geometric", "hdq", "hdq_fast"]

    for method in methods:
        print("\n=========================================")
        print(f"Running method: {method}")
        print("=========================================")

        result = run_one_case(
            gamma_O=1.0,
            gamma_T=1.0,
            disturbance_scale=1.0,
            dt=0.005,
            total_time=8.0,
            damping=1e-3,
            save_prefix=f"disturbance_{method}",
            jacobian_method=method
        )

        print(f"method = {method}")
        print(f"gamma_O_sim = {result['gamma_O_sim']:.4f}")
        print(f"gamma_T_sim = {result['gamma_T_sim']:.4f}")
        print(f"Final orientation error = {result['eO'][-1]:.6f}")
        print(f"Final translation error = {result['eT'][-1]:.6f}")
        summarize_runtime(result)


if __name__ == "__main__":
    main()