import pandas as pd
import matplotlib.pyplot as plt

# Load the data
file_no_extra = "execution_times_no_extra.csv"
file_extra = "execution_times_extra.csv"

df_no_extra = pd.read_csv(file_no_extra)
df_extra = pd.read_csv(file_extra)

# Create the plot
plt.figure(figsize=(12, 6), dpi=100)  # Increase figure size and resolution

# Plot both datasets with clearer visibility
plt.plot(df_no_extra["Iteration"], df_no_extra["ExecutionTime(ms)"], 'bo-', markersize=3, alpha=0.7, label="Without Extra Load")
plt.plot(df_extra["Iteration"], df_extra["ExecutionTime(ms)"], 'rx-', markersize=3, alpha=0.7, label="With Extra Load")

# Improve Labels and Title
plt.xlabel("Iteration", fontsize=14)
plt.ylabel("Execution Time (ms)", fontsize=14)
plt.title("Execution Time Comparison: With and Without Extra Load", fontsize=16)
plt.legend(fontsize=12)

# Improve Grid for readability
plt.grid(True, linestyle="--", alpha=0.6)

# Show the plot
plt.show()
