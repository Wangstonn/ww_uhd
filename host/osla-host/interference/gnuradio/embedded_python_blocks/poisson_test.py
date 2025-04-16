import numpy as np
import matplotlib.pyplot as plt

# Parameters
lambda_rate = 200  # Arrivals per second
num_samples = 10000  # Number of samples

# Generate uniform random numbers
U1= np.random.uniform(0, 1, num_samples)
U2= np.random.uniform(size=num_samples)

# Apply inverse transform sampling to get an exponential distribution
exp_samples1 = -np.log(U1) / lambda_rate
exp_samples2 = -np.log(U2) / lambda_rate

# Plot histogram of generated exponential data
plt.figure(figsize=(8, 6))
plt.hist(exp_samples1, bins=50, density=True, alpha=0.6, label='Generated Data', color='b', edgecolor='black')

# Plot theoretical exponential PDF
x_vals = np.linspace(0, max(exp_samples1), 100)
pdf_vals = lambda_rate * np.exp(-lambda_rate * x_vals)
plt.plot(x_vals, pdf_vals, 'r', linewidth=2, label='Theoretical Exp PDF')

# Labels and title
plt.xlabel('Interarrival Time (seconds)')
plt.ylabel('Probability Density')
plt.title('Exponential Distribution Generated from Uniform Distribution')
plt.legend()
plt.grid(True)
plt.show()

# Plot histogram of generated exponential data
plt.figure(figsize=(8, 6))
plt.hist(exp_samples2, bins=50, density=True, alpha=0.6, label='Generated Data', color='b', edgecolor='black')

# Plot theoretical exponential PDF
x_vals = np.linspace(0, max(exp_samples2), 100)
pdf_vals = lambda_rate * np.exp(-lambda_rate * x_vals)
plt.plot(x_vals, pdf_vals, 'r', linewidth=2, label='Theoretical Exp PDF')

# Labels and title
plt.xlabel('Interarrival Time (seconds)')
plt.ylabel('Probability Density')
plt.title('Exponential Distribution Generated from Uniform Distribution')
plt.legend()
plt.grid(True)
plt.show()

# Compute and display the mean
simulated_mean1 = np.mean(exp_samples1)
theoretical_mean = 1 / lambda_rate

# Compute and display the mean
simulated_mean2 = np.mean(exp_samples2)

print(f"Simulated Mean: {simulated_mean1:.6f} sec")
print(f"Simulated Mean: {simulated_mean2:.6f} sec")
print(f"Theoretical Mean: {theoretical_mean:.6f} sec")