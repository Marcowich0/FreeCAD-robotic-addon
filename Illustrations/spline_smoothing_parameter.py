import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# Create a subfolder "save" in the current working directory if it doesn't exist
save_dir = "data"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# Generate a "hilly" landscape using a sin/cos function.
x = np.linspace(0, 10, 50)
y = np.sin(x) + 0.5 * np.cos(3 * x)
points = np.vstack((x, y))  # shape (2, N)

# Save original points to CSV file in the "save" folder.
original_data = np.column_stack((x, y))
orig_csv = os.path.join(save_dir, "original_points.csv")
np.savetxt(orig_csv, original_data, delimiter=",", header="x,y", comments="")

# Define smoothing parameters to test.
smoothing_params = [0, 0.01, 0.1, 1]
labels = ["y0", "y0.01", "y0.1", "y1"]

# Fit splines and evaluate on a fine grid.
num_sample = 300
u_fine = np.linspace(0, 1, num_sample)
spline_results = {}

for s, label in zip(smoothing_params, labels):
    # Fit spline; splprep expects data as a list/array of arrays.
    tck, u = splprep(points, s=s)
    out = splev(u_fine, tck)
    # Convert output to a (num_sample, 2) array.
    spline_results[label] = np.column_stack((out[0], out[1]))

# Use the x-coordinates from the spline with s=0 as representative.
x_spline = spline_results["y0"][:, 0]

# Create a DataFrame to hold the spline data.
df = pd.DataFrame({"x": x_spline})
for label in labels:
    df[label] = spline_results[label][:, 1]

# Save the spline data to CSV file in the "save" folder.
spline_csv = os.path.join(save_dir, "spline_data.csv")
df.to_csv(spline_csv, index=False)

print("CSV files generated:")
print(" -", orig_csv)
print(" -", spline_csv)

# Optionally, also plot the data
plt.figure(figsize=(8,6))
plt.plot(points[0], points[1], 'ko-', label='Data points')
colors = ['r', 'g', 'b', 'm']
for s, color, label in zip(smoothing_params, colors, labels):
    plt.plot(spline_results[label][:, 0], spline_results[label][:, 1],
             color, label=f'Spline s={s}')
plt.legend()
plt.title("Spline Fits on a Hilly Landscape")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
