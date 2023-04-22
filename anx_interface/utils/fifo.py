class FiFo:
    def __init__(self, in_pipe, out_pipe):
        self.in_pipe = in_pipe
        self.out_pipe = out_pipe

    # blocking call
    def send_msg(self, msg: str) -> str:
        with open(self.in_pipe, "w") as fifo_in:
            fifo_in.write(f"{msg}\n")

        data = ""
        with open(self.out_pipe) as fifo_out:
            while True:
                read_data = fifo_out.read()
                # reached EOF
                if len(read_data) == 0:
                    return data
                data = read_data

def parse_msg(msg: str):
    """
    Utility function to parse messages from fifo based rpc server

    Arguments:
        msg -- in the format of (0 or 1);(data here). Separator is ';'

    Returns:
        a tuple which consists of 1st element True/False, rest of the elements are data
    """

    tokens = msg.split(";")
    tokens[0] = tokens[0] == "1"
    return tuple(tokens)