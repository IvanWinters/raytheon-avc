import datetime

def get_timestamped_filename(prefix, extension):
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{prefix}_{timestamp}.{extension}"

def save_text_log(log_data, prefix="test_results"):
    filename = get_timestamped_filename(prefix, "log")
    with open(filename, "w") as f:
        f.write(log_data)
    return filename
