import matplotlib.pyplot as plt

# Points for ABCDE
points = {
    "A": (0.93, 0.28),
    "B": (0.83, 1.19),
    "C": (1.41, 1.38),
    "D": (1.75, 0.44),
    "E": (1.29, 0.22)
}

# Points for LM1 and LM2
landmarks = {
    "LM1": (0.31, 0.4),
    "LM2": (0.32, 1.4)
}

# Extract x and y values for ABCDE
x_abcde = [points[key][0] for key in points]
y_abcde = [points[key][1] for key in points]

# Extract x and y values for LM1 and LM2
x_landmarks = [landmarks[key][0] for key in landmarks]
y_landmarks = [landmarks[key][1] for key in landmarks]

# Plot the points and lines
plt.figure(figsize=(10, 10))
plt.plot(x_abcde, y_abcde, marker='o', color='b', label='ABCDE Path')
plt.scatter(x_landmarks, y_landmarks, color='r', label='Landmarks LM1 & LM2')

# Annotate points
for label, (x, y) in points.items():
    plt.text(x, y, f" {label}", fontsize=10)
for label, (x, y) in landmarks.items():
    plt.text(x, y, f" {label}", fontsize=10)

# Formatting the plot
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(0,2)
plt.ylim(0,2)
plt.title('Path ABCDE and Landmarks LM1 & LM2')
plt.legend()
plt.grid()
plt.show()
