import h5py
import numpy as np

file_path = "data/run_20260331_121635.h5"


def inspect_dataset(name, data):
    print(f"\n=== {name} ===")

    if not hasattr(data, "shape"):
        print("Not a dataset")
        return

    print("shape:", data.shape)
    print("dtype:", data.dtype)

    # Load a small sample (avoid loading huge data)
    sample = data[:5]

    print("\nFirst 5 entries:")
    print(sample)

    # If numeric, show stats
    if np.issubdtype(data.dtype, np.number):
        try:
            arr = data[:]
            print("\nStats:")
            print(" min:", np.min(arr))
            print(" max:", np.max(arr))
            print(" mean:", np.mean(arr))

            # Show unique values if small range
            unique_vals = np.unique(arr)
            if len(unique_vals) < 50:
                print(" unique:", unique_vals)
            else:
                print(" unique: too many to display")

        except Exception as e:
            print(" (Skipping full stats due to size)", e)


def summarize_critics(f):
    print("\n====== CRITIC SUMMARY ======")
    scores = f["scores"]
    scalars = f["scalars"]

    num_samples = scores.shape[0]
    # Last column of scalars assumed to store NUM_CRITICS
    num_critics = int(scalars[0, -1])
    critic_length = scores.shape[1] // num_critics

    print("NUM_CRITICS:", num_critics)
    print("Scores per critic:", critic_length)
    print("Number of samples:", num_samples)

    # Show min/max/mean for each critic
    for i in range(num_critics):
        start = i * critic_length
        end = start + critic_length
        critic_scores = scores[:, start:end]

        print(f"\n--- Critic {i+1} ---")
        print("First row (first 10 values):", critic_scores[0, :10])
        print("Min:", np.min(critic_scores))
        print("Max:", np.max(critic_scores))
        print("Mean:", np.mean(critic_scores))

    # Show some sample timestamps / metadata
    print("\nSample scalar metadata (first 5 rows):")
    print(scalars[:5])


def main():
    with h5py.File(file_path, "r") as f:
        print("====== FILE STRUCTURE ======")

        # Inspect all datasets
        def visitor(name, obj):
            if isinstance(obj, h5py.Dataset):
                inspect_dataset(name, obj)

        f.visititems(visitor)

        # Summarize critics
        summarize_critics(f)


if __name__ == "__main__":
    main()
