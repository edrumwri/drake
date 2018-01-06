import sys
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) <= 1:
  print('Syntax: plot_rod_state <state index>')

# Get the index
index = int(sys.argv[1])

# Plot data from the piecewise DAE file
y = np.loadtxt('state.output.pDAE')
t = y[:,0]
x = y[:,index]
plt.plot(t, x, 'b', label='pDAE')

# Plot data from the compliant file
y = np.loadtxt('state.output.compliant')
t = y[:,0]
x = y[:,index]
plt.plot(t, x, 'r', label='compliant')

# Plot data from the timestepping output file
y = np.loadtxt('state.output.timestepping')
t = y[:,0]
x = y[:,index]
plt.plot(t, x, 'k', label='time-stepping')

# Set up the legend.
plt.legend(loc='upper right', shadow=True)



# Display the plot
plt.show()

