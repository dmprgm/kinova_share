import matplotlib.pyplot as plt
import numpy as np

# Define the data
categories = ['TwistX', 'TwistY', 'TwistZ', 
              'AvgVelocity', 'MaxVelocity', 'AvgAccel', 'MaxAccel', 
              'RangeY', 'RangeZ', 'TimeElapsed', 'PathLengthDifference']

A = [0.109, 0.01, -0.071, 1.571, 6.023, 24.457, 276.869, 
     0.063, 0.35, 3.829, 0.33]

B = [0.054, 0.004, -0.052, 0.656, 3.448, 8.917, 116.25, 
     0.049, 0.302, 6.239, 0.18]

# Number of variables we're plotting
num_vars = len(categories)

# Compute angle for each category
angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()

# Make the plot circular by appending the start to the end
A += A[:1]
B += B[:1]
angles += angles[:1]

# Plotting
fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))

ax.fill(angles, A, color='blue', alpha=0.25)
ax.fill(angles, B, color='red', alpha=0.25)

ax.plot(angles, A, color='blue', linewidth=2, label='Condition A')
ax.plot(angles, B, color='red', linewidth=2, label='Condition B')

# Add labels to the axes
ax.set_yticklabels([])
ax.set_xticks(angles[:-1])
ax.set_xticklabels(categories, fontsize=10)

# Add a title and legend
plt.title('Spider Chart Comparing Conditions A and B', size=15, color='black', y=1.1)
ax.legend(loc='upper right', bbox_to_anchor=(1.1, 1.1))

plt.show()
