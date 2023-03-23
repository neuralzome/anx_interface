#!/usr/bin/env python3

from enum import Enum
import numpy as np
import zmq

import anx_proto.python.common_pb2 as common_pb2
import anx_proto.python.model_pb2 as model_pb2

tflite_numpy_dtype_map = {
    1: np.float32,
    2: np.int32,
    3: np.uint8,
    4: np.int64,
    7: np.int16,
    9: np.int8,
    10: np.float16,
    11: np.float64,
    13: np.uint64,
    16: np.uint32
}

class DeviceType(Enum):
    CPU = 1
    GPU = 2
    DSP = 3

class TfliteInterface:
    def __init__(self, device_type):
        """
        Arguments:
            device_type -- select from DeviceType enum
        """
        self.ctx = zmq.Context()
        self.socket = self.ctx.socket(zmq.REQ)

        if device_type == DeviceType.CPU:
            self.socket.connect(f"ipc:///ipc/cpu")
        elif device_type == DeviceType.GPU:
            self.socket.connect(f"ipc:///ipc/gpu")
        elif device_type == DeviceType.DSP:
            self.socket.connect(f"ipc:///ipc/dsp")

        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        self.model_loaded = False
        self.model = None

        self.input = None
        self.input_shape = None
        self.input_dtype = None

        self.output_shape = None
        self.output_dtype = None
        self.output = None

    def load_model(self, path_to_model):
        """
        load tflite model
        Arguments:
            path_to_model -- path to the tflite model
        Returns:
            True if successful
        """

        with open(path_to_model, mode="rb") as model_fd:
            self.model = model_fd.read()
            request = common_pb2.Payload()
            request.payload = self.model
            request_bytes = request.SerializeToString()
            self.socket.send_multipart([b"LoadGpuModel", request_bytes])

            events = self.poller.poll(2000)
            if events:
                model_meta_rep = model_pb2.ModelMeta()
                std_rep = common_pb2.StdResponse()
                std_rep_bytes, model_meta_rep_bytes = self.socket.recv_multipart()
                std_rep.ParseFromString(std_rep_bytes)
                if std_rep.success:
                    self.model_loaded = True
                    model_meta_rep.ParseFromString(model_meta_rep_bytes)

                    # Populate input/output shape & dtype
                    self.input_shape = tuple(model_meta_rep.input_dims)
                    self.input_dtype = tflite_numpy_dtype_map[model_meta_rep.input_dtype]
                    print(f"Input shape: {self.input_shape}, input_dtype: {self.input_dtype}")

                    self.output_shape = tuple(model_meta_rep.output_dims)
                    self.output_dtype = tflite_numpy_dtype_map[model_meta_rep.output_dtype]
                    print(f"Output shape: {self.output_shape}, output_dtype: {self.output_dtype}")
                    return True
                else:
                    print(std_rep.message)
        return False

    def unload_model(self):
        """
        unload model
        Returns:
            True if successful
        """
        request = common_pb2.Empty()
        request_bytes = request.SerializeToString()
        self.socket.send_multipart([b"UnloadGpuModel", request_bytes])

        events = self.poller.poll(2000)
        if events:
            rep = common_pb2.StdResponse()
            rep_bytes = self.socket.recv()
            rep.ParseFromString(rep_bytes)
            if rep.success:
                self.model_loaded = False
                self.model = None

                self.input = None
                self.input_shape = None
                self.input_dtype = None

                self.output_shape = None
                self.output_dtype = None
                self.output = None
                return True
            else:
                print(rep.message)
        return False

    def set_input(self, input):
        """
        set input to the model
        Arguments:
            input -- numpy array of shape and type as acceptable by tflite model
        Returns:
            True if successful
        """
        if not self.model_loaded:
            print("Load model first!!")
            return False

        if not input.shape == self.input_shape:
            print(f"Input shape should be {self.input_shape}")
            return False

        if not input.dtype == self.input_dtype:
            print(f"Input dtype should be {self.input_dtype}")
            return False

        self.input = input
        return True

    def invoke(self):
        """
        invoke the model
        Returns:
            True if successful
        """

        if not self.model_loaded:
            print("Load model first!!")
            return False

        req = common_pb2.Payload()
        req.payload = self.input.tobytes()
        req_bytes = req.SerializeToString()
        self.socket.send_multipart([b"InvockGpuModel", req_bytes])

        events = self.poller.poll(2000)
        if events:
            rep = common_pb2.Payload()
            rep_bytes = self.socket.recv()
            rep.ParseFromString(rep_bytes)

            self.output = np.frombuffer(rep.payload, dtype=self.output_dtype)
            self.output = self.output.reshape(self.output_shape)
            return True
        return False

    def get_output(self):
        """
        Returns:
            model output as numpy array 
        """
        if not self.model_loaded:
            print("Load model first!!")
            return None
        return self.output
