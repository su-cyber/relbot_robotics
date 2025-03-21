import pandas as pd
import matplotlib.pyplot as plt

# Read and process data
df = pd.read_csv('jitter_log.csv')

# Calculate execution time between consecutive entries (in ns)
timestamps = df['Timestamp (s)'] * 1e9 + df['Timestamp (ns)']
execution_times = timestamps.diff().dropna().reset_index(drop=True)

# Create plot
plt.figure(figsize=(14, 8))

# Main plot - Execution Times
ax1 = plt.subplot(2, 1, 1)
ax1.plot(execution_times, linewidth=0.7, color='tab:blue', label='Execution Time')
ax1.set_title('Execution Times and Jitter Over Iterations', fontsize=14)
ax1.set_ylabel('Execution Time (ns)', color='tab:blue')
ax1.grid(True, alpha=0.3)
ax1.set_xlim(0, 5000)

# Add execution time statistics
mean_et = execution_times.mean()
ax1.axhline(mean_et, color='red', linestyle='--', linewidth=1, 
          label=f'Mean: {mean_et:.1f} ns')
ax1.legend(loc='upper right')

# Secondary plot - Jitter
ax2 = plt.subplot(2, 1, 2, sharex=ax1)
ax2.plot(df['Jitter (ns)'], linewidth=0.7, color='tab:orange', alpha=0.8)
ax2.set_xlabel('Iteration Number', fontsize=12)
ax2.set_ylabel('Jitter (ns)', color='tab:orange')
ax2.grid(True, alpha=0.3)

# Add statistics box
stats_text = f"""Execution Time Statistics:
Max: {execution_times.max():.1f} ns
Min: {execution_times.min():.1f} ns
Mean: {mean_et:.1f} ns
Std Dev: {execution_times.std():.1f} ns"""
ax1.text(0.98, 0.75, stats_text, transform=ax1.transAxes,
        verticalalignment='top', horizontalalignment='right',
        bbox=dict(facecolor='white', alpha=0.8))

plt.tight_layout()
plt.show()