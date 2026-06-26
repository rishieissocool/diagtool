import functools
import os
import sys

def redirect_print_to_log(print_is_enabled=True):
    log = ""
    def outer(func):
        @functools.wraps
        def inner(*args, **kwargs):
            # make a copy of sys.stdout and sys.stderr
            stdout = sys.stdout
            stderr = sys.stderr
            
            with open('/dev/null') as f:
                    sys.stdout = f
                    sys.stderr = f

            if print_is_enabled:
                # set flag to make a new log file
                flag = "a"

                # if the log file already exists, append log to it
                if os.path.exists(log):
                    flag = "w+"

                # open up the log file for writing
                # doesn't check if we have the permissions to write to it but
                with open(log, flag) as f:
                    # change sys.stdout and sys.stderr to log file
                    sys.stdout = f
                    sys.stderr = f

                    # run func
            func(*args, **kwargs)

            # reset sys.stdout and sys.stderr
            sys.stdout = stdout
            sys.stderr = stderr
        return inner
    return outer