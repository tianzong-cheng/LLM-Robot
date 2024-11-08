# Background

I am working on a project that interprets human language input to control a mechanical arm, enabling it to complete tasks as instructed. You need to break the task into some subtasks of picking and placing, and write a Python code snippet to specify the pick-place subtasks.

You will be given a template of the code snippet. Please preserve the formatting and overall template that I provide. Note that `initial_position` and `target_position` correspond one-by-one, and the order of the positions specified is the order the pick-place tasks executed.

Don't include any unknown information in your code. If you think some necessary information is missing, please output error information instead of code.

For my convenience, please output the code only without any extra explanation.

# Code Snippet Template

```python
subtask_num = N  # Replace N with the number of tasks
initial_position = [
    # N lines of np.array([x, y, z])
    np.array([x_i, y_i, z_i]),
]
target_position = [
    # N lines of np.array([x, y, z])
    np.array([x_t, y_t, z_t]),
]
```

# Input-output Example Pairs

## Example 1

Task: Put the red cup in the cup holder.

Object positions:

- red cup: `[0.5, -0.2, 0]`
- cup holder: `[0.3, -0.4, 0.1]`

```python
subtask_num = 1
initial_position = [
    np.array([0.5, -0.2, 0]),
]
target_position = [
    np.array([0.3, -0.4, 0.1]),
]
```

## Example 2

Task: Stack the objects on the table in the order of red cube, black cube and yellow cylinder, from top to bottom.

Object positions:

- red cube: `[0.5, -0.2, 0]`
- black cube: `[0.3, -0.4, 0.1]`
- yellow cylinder: `[0.4, -0.3, 0]`

```python
subtask_num = 2
initial_position = [
    np.array([0.3, -0.4, 0.1]),
    np.array([0.5, -0.2, 0]),
]
target_position = [
    np.array([0.4, -0.3, 0 + 0.05]),
    np.array([0.4, -0.3, 0 + 0.05 * 2]),
]
```

### Explanation

When stacking several objects, you need to stack from bottom up. For example, stacking in the order of red cube, black cube and yellow cylinder, from top to bottom, you need to first put the black cube on the second layer, then the red cube on the top layer. The yellow cylinder, which is the object at the bottom, should not be moved.

Note that a small offset in the z-axis needs to be added. Since you don't need to move the object at the bottom, only two subtasks are needed.

# Your Task
