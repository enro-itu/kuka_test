import pandas as pd

def get_workspace_extremes(file_path):
    """
    Reads a CSV file containing robot workspace coordinates and returns
    the min and max values for x, y, and z.
    """
    try:
        # Load the dataset
        df = pd.read_csv(file_path)
        
        # Normalize column names to lowercase and strip whitespace
        # This handles variations like " X", "x", "X " to ensure we match correctly
        df.columns = df.columns.str.strip().str.lower()
        
        results = {}
        target_axes = ['x', 'y', 'z']
        
        # Check if basic columns exist
        missing_cols = [col for col in target_axes if col not in df.columns]
        if len(missing_cols) == 3:
             return "Error: Could not find x, y, or z columns in the CSV."

        for axis in target_axes:
            if axis in df.columns:
                results[axis] = {
                    'min': df[axis].min(),
                    'max': df[axis].max(),
                    'span': df[axis].max() - df[axis].min()
                }
        
        return results

    except Exception as e:
        return f"An error occurred: {str(e)}"
    

limits = get_workspace_extremes('C:/Users/arife/Desktop/İTÜ/ENRO/envelope/rk_demo/data/valid_workspace.csv')
print(limits)