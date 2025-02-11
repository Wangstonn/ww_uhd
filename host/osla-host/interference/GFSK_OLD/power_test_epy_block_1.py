import numpy as np
from gnuradio import gr

class logging_block(gr.sync_block):
    """
    Logs complex input data to a file and stops processing after a specified number of samples.
    Signals the GNU Radio runtime to terminate by returning -1 once the sample limit is reached.
    """
    def __init__(self, file_path="output.csv", sample_limit=1000):
        gr.sync_block.__init__(
            self,
            name="Logging Block",
            in_sig=[np.complex64],  # Input: complex data
            out_sig=[np.complex64]  # Output: complex data
        )
        self.file_path = file_path
        self.sample_limit = sample_limit
        self.sample_count = 0
        self.file = open(self.file_path, 'w')

    def work(self, input_items, output_items):
        in_data = input_items[0]   # Input data

        # Determine how many samples to process before reaching the limit
        remaining_samples = self.sample_limit - self.sample_count
        n_samples = min(len(in_data), remaining_samples)

        # Log data to file
        for i in range(n_samples):
            self.file.write(f"{in_data[i].real}, {in_data[i].imag}\n")  # Write real and imaginary parts

        # Update sample count
        self.sample_count += n_samples

        # Check if the sample limit has been reached
        if self.sample_count >= self.sample_limit:
            self.file.close()  # Close the file to ensure all data is written
            return -1  # Signal GNU Radio to stop processing

        # Return the number of processed samples
        return n_samples

    def __del__(self):
        # Ensure the file is closed properly if the block is deleted
        if not self.file.closed:
            self.file.close()
