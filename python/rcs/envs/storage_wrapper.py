import operator
from typing import Any, Optional
from uuid import uuid4
from queue import Queue
import pyarrow.dataset as ds
import pyarrow as pa
import numpy as np
from itertools import chain
import simplejpeg
from concurrent.futures import ThreadPoolExecutor, wait
import gymnasium as gym


class StorageWrapper(gym.Wrapper):
    QueueSentinel = None

    def __init__(
        self,
        env: gym.Env,
        base_dir: str,
        batch_size: int,
        schema: Optional[pa.Schema] = None,
        start_record: bool = False,
        basename_template: Optional[str] = None,
        max_rows_per_group: Optional[int] = None,
        max_rows_per_file: Optional[int] = None,
    ):
        """
        Asynchronously log environment transitions to a Parquet
        dataset on disk.

        Observation handling:
        - Expects observations to be dictionaries.
        - RGB camera frames are JPEG-encoded.
        - Numpy arrays with ndim > 1 inside the observation dict are flattened
            in-place, and their original shapes are stored alongside as
            ``"<key>_shape"``. Nested dicts are traversed recursively.
        - Lists/tuples of arrays are not supported.
        - ``close()`` must be called to flush the final batch.

        Parameters
        ----------
        env : gym.Env
            The environment to wrap.
        base_dir : str
            Output directory where the Parquet dataset will be written.
        batch_size : int
            Number of transitions to accumulate before flushing a RecordBatch
            to the writer queue.
        schema : Optional[pa.Schema], default=None
            Optional Arrow schema to enforce for all batches. If None, the schema
            is inferred from the first flushed batch and then reused.
        basename_template : Optional[str], default=None
            Template controlling Parquet file basenames. Passed through to
            ``pyarrow.dataset.write_dataset``.
        max_rows_per_group : Optional[int], default=None
            Maximum row count per Parquet row group. Passed through to
            ``pyarrow.dataset.write_dataset``.
        max_rows_per_file : Optional[int], default=None
            Maximum row count per Parquet file. Passed through to
            ``pyarrow.dataset.write_dataset``.
        """

        super().__init__(env)
        self.base_dir = base_dir
        self.batch_size = batch_size
        self.schema = schema
        self.basename_template = basename_template
        self.max_rows_per_group = max_rows_per_group
        self.max_rows_per_file = max_rows_per_file
        self.buffer = []
        self.step_cnt = 0
        self._pause = True
        self._success = start_record
        self._prev_action = None
        self.thread_pool = ThreadPoolExecutor()
        self.queue = Queue(maxsize=2)
        self.uuid = uuid4()
        self._writer_future = self.thread_pool.submit(self._writer_worker)

    def _generator_from_queue(self):
        while (batch := self.queue.get()) is not self.QueueSentinel:
            yield batch

    def _writer_worker(self):
        gen = self._generator_from_queue()
        first = next(gen)
        ds.write_dataset(
            data=chain([first], gen),
            base_dir=self.base_dir,
            format="parquet",
            schema=self.schema,
            existing_data_behavior="overwrite_or_ignore",
            basename_template=self.basename_template,
            max_rows_per_group=self.max_rows_per_group,
            max_rows_per_file=self.max_rows_per_file,
            partitioning=ds.partitioning(
                schema=pa.schema(fields=[pa.field("uuid", pa.binary(36))]),
                flavor="filename",
            ),
        )

    def _flush(self):
        batch = pa.RecordBatch.from_pylist(self.buffer, schema=self.schema)
        if self.schema is None:
            self.schema = batch.schema
        self.queue.put(batch)
        self.buffer.clear()

    def _flatten_arrays(self, d: dict[str, Any]):
        # NOTE: list / tuples of arrays not supported
        updates = {}
        for k, v in d.items():
            if isinstance(v, dict):
                self._flatten_arrays(v)
            elif isinstance(v, np.ndarray) and len(v.shape) > 1:
                d[k] = v.flatten()
                updates[f"{k}_shape"] = v.shape
        d.update(updates)

    def _encode_images(self, obs: dict[str, Any]):
        _ = [
            *self.thread_pool.map(
                lambda cam: operator.setitem(
                    obs["frames"][cam]["rgb"],
                    "data",
                    simplejpeg.encode_jpeg(np.ascontiguousarray(obs["frames"][cam]["rgb"]["data"])),
                ),
                obs["frames"],
            )
        ]

    def step(self, action):
        # NOTE: expects the observation to be a dictionary
        if self._writer_future.done():
            exc = self._writer_future.exception()
            assert exc is not None
            msg = "Writer thread failed"
            raise RuntimeError(msg) from exc
        obs, reward, terminated, truncated, info = self.env.step(action)
        if not self._pause:
            assert isinstance(obs, dict)
            self._encode_images(obs)
            self._flatten_arrays(obs)
            if "success" in info and info["success"]:
                self.success()
            self.buffer.append({"obs": obs, "reward": reward, "step": self.step_cnt, "uuid": self.uuid.bytes, "success": self._success, "action": self._prev_action})
            self._prev_action = action
            self.step_cnt += 1
            if len(self.buffer) == self.batch_size:
                self._flush()
        return obs, reward, terminated, truncated, info

    def success(self):
        self._success = True

    def stop_record(self):
        self._pause = True
        if len(self.buffer) > 0:
            self._flush()

    def start_record(self):
        self._pause = False

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        if len(self.buffer) > 0:
            self._flush()
        self._pause = True
        self._success = False
        self._prev_action = None
        obs, info = self.env.reset()
        self.step_cnt = 0
        self.uuid = uuid4()
        return obs, info

    def close(self):
        if len(self.buffer) > 0:
            self._flush()
        self.queue.put(self.QueueSentinel)
        wait([self._writer_future])
