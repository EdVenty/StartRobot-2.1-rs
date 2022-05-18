from asyncio.subprocess import Process
import base64
from dataclasses import dataclass
import json
from queue import Empty, Queue
import subprocess
from typing import Any, Dict, List, Union
from loguru import logger
from threading import Thread
import websockets
import asyncio
import sys
import os
from enum import Enum

LOCAL_SERVICE_KEYWORD = 'service'
FILES_FILE = "files.json"
FILES_DIRECTORY = "files"
logger.remove()
logger.add(sys.stdout, level='DEBUG')

class FilePropertyType(Enum):
	Program = 0
	RunOnBoot = 1
	User = 2

class FileMessageType(Enum):
	AddFile = 0
	RemoveFile = 1
	Mark = 2
	GetFiles = 3
	RunFile = 4
	StopFile = 5

class FileToRobotMessage:
	event: FileMessageType

@dataclass
class AddFileMessage(FileToRobotMessage):
	file_name: str
	file_content: str
	event = FileMessageType.AddFile

@dataclass
class RemoveFileMessage(FileToRobotMessage):
	file_name: str
	event = FileMessageType.RemoveFile

@dataclass
class MarkFileMessage(FileToRobotMessage):
	file_name: str
	file_property_type: FilePropertyType
	file_property_value: Any
	event = FileMessageType.Mark

@dataclass
class GetFilesMessage(FileToRobotMessage):
	event = FileMessageType.GetFiles

@dataclass
class RunFileMessage(FileToRobotMessage):
	file_name: str

@dataclass
class StopFileMessage(FileToRobotMessage):
	file_name: str

class FileProcess:
	def __init__(self, filename: str, python_command: str = 'python', reader_queue = Queue()) -> None:
		self.python_command = python_command
		self.filename = filename
		self.proc = None
		self.is_running = False
		self.reader_task = None
		self.reader_queue = reader_queue

	async def reader(self):
		while True:
			message = (await self.proc.stdout.readline()).decode('utf-8')
			if self.proc.returncode != None:
				if self.is_running:
					await self.stop()
				if len(message) > 0:
					self.reader_queue.put_nowait(message)
			else:
				self.reader_queue.put_nowait(message)
			await asyncio.sleep(0.1)

	async def run(self):
		self.is_running = True
		self.proc = await asyncio.create_subprocess_exec(self.python_command, f"{FILES_DIRECTORY}/{self.filename}", stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.STDOUT)
		self.reader_task = asyncio.create_task(self.reader())
		
	async def stop(self):
		self.is_running = False
		try:
			self.proc.terminate()
		except ProcessLookupError:
			logger.warning("ProcessLookupError when trying to stop process.")
		await asyncio.sleep(3)
		self.reader_task.cancel()

class File:
	def __init__(self, file_name = None, properties = {}, is_running = False) -> None:
		self.file_name: str = file_name
		self.properties: Dict[FilePropertyType, Any] = {FilePropertyType(int(key)): properties[key] for key in properties}
		self.is_running = is_running

	def to_dict(self):
		return {
			"file_name": self.file_name,
			"properties": {key.value: self.properties[key] for key in self.properties},
			"is_running": self.is_running
		}

	def to_message_dict(self):
		return {
			"FileName": self.file_name,
			"FileProperties": {key.value: self.properties[key] for key in self.properties},
			"IsRunning": self.is_running
		}

class FileHost:
	def __init__(self) -> None:
		self._file_channel_thread = None
		self.ws = None
		self.processes: Dict[str, FileProcess] = {}
		self.python_command = 'python'
		self.reader_queue = Queue()
		if not os.path.exists(FILES_DIRECTORY):
			os.makedirs(FILES_DIRECTORY)
		if not os.path.exists(FILES_FILE):
			with open(FILES_FILE, 'x') as file:
				file.write(json.dumps({})) 
		self.choose_python_command()

	@logger.catch()
	def validate_files(self):
		info = self.read_files_info()
		for name in info:
			info[name].is_running = False
		self.write_files_info(info)

	@logger.catch()
	def choose_python_command(self):
		if subprocess.check_output(['python', '--version']).decode('utf-8').startswith('Python 3'):
			self.python_command = 'python'
		else:
			self.python_command = 'python3'
		logger.debug(f"Python command detected is '{self.python_command}'.")

	@logger.catch()
	async def send_update_files(self):
		information = self.read_files_info()
		await self.ws.send(json.dumps({
			"Files": [information[key].to_message_dict() for key in information]
		}))

	@logger.catch()
	def read_files_info(self) -> Dict[str, File]:
		with open(FILES_FILE, 'r', encoding='utf-8') as file:
			raw = json.loads(file.read())
			return {key: File(**raw[key]) for key in raw}

	@logger.catch()
	def write_files_info(self, files: Dict[str, File]):
		with open(FILES_FILE, 'w', encoding='utf-8') as file:
			file.write(json.dumps({key: files[key].to_dict() for key in files}))

	@logger.catch()
	async def on_add_file_event(self, event: AddFileMessage):
		logger.debug("Add file event got.")
		path = f"{FILES_DIRECTORY}/{event.file_name}"
		if os.path.exists(path):
			logger.debug(f"Removing existing file: {path}.")
			os.remove(path)
		logger.debug(f"Making new file: {path}.")
		with open(path, 'xb') as file:
			file.write(base64.b64decode(event.file_content))
		logger.debug("Updating files info.")
		information = self.read_files_info()
		information.update({event.file_name: File(event.file_name)})
		self.write_files_info(information)
		await self.send_update_files()

	@logger.catch()
	async def on_remove_file_event(self, event: RemoveFileMessage):
		logger.debug("Remove file event got.")
		path = f"{FILES_DIRECTORY}/{event.file_name}"
		if os.path.exists(path):
			logger.debug(f"Removing file: {path}")
			os.remove(path)
		logger.debug("Updating files info.")
		information = self.read_files_info()
		information.pop(event.file_name)
		self.write_files_info(information)
		await self.send_update_files()

	@logger.catch()
	async def on_mark_file_event(self, event: MarkFileMessage):
		logger.debug("Mark file event got.")
		information = self.read_files_info()
		if event.file_name in information:
			information[event.file_name].properties[FilePropertyType(event.file_property_type)] = event.file_property_value
			self.write_files_info(information)
		await self.send_update_files()
	
	@logger.catch()
	async def on_get_files_event(self, event: GetFilesMessage):
		logger.debug("Update files event got.")
		await self.send_update_files()

	@logger.catch()
	async def on_run_file(self, event: RunFileMessage):
		information = self.read_files_info()
		if event.file_name in information and event.file_name not in self.processes:
			fileproc = FileProcess(event.file_name, self.python_command, self.reader_queue)
			self.processes.update({event.file_name: fileproc})
			await fileproc.run()
			information[event.file_name].is_running = True
			self.write_files_info(information)
			logger.debug(f"File {event.file_name} started.")
		await self.send_update_files()

	@logger.catch()
	async def on_stop_file(self, event: StopFileMessage):
		information = self.read_files_info()
		if event.file_name in information and event.file_name in self.processes:
			try:
				await self.processes[event.file_name].stop()
			except ProcessLookupError:
				logger.warning(f"ProcessLookupError occurred when trying to terminate file process {event.file_name}.")
			self.processes.pop(event.file_name)
			information[event.file_name].is_running = False
			self.write_files_info(information)
			logger.debug(f"File {event.file_name} stopped (process terminated).")
		elif event.file_name in information and information[event.file_name].is_running:
			information[event.file_name].is_running = False
			self.write_files_info(information)
			logger.warning(f"File process close event found stopped process, but file was marked as running.")
		await self.send_update_files()

	async def route_event(self, value: Dict):
		event = FileMessageType(value['Event'])
		if event == FileMessageType.AddFile:
			await self.on_add_file_event(AddFileMessage(
				file_name=value['FileName'],
				file_content=value['FileContent']
			))
		elif event == FileMessageType.RemoveFile:
			await self.on_remove_file_event(RemoveFileMessage(
				file_name=value['FileName']
			))
		elif event == FileMessageType.Mark:
			await self.on_mark_file_event(MarkFileMessage(
				file_name=value['FileName'],
				file_property_type=FilePropertyType(value['FilePropertyType']),
				file_property_value=value['FilePropertyValue']
			))
		elif event == FileMessageType.GetFiles:
			await self.on_get_files_event(GetFilesMessage())
		elif event == FileMessageType.RunFile:
			await self.on_run_file(RunFileMessage(file_name=value['FileName']))
		elif event == FileMessageType.StopFile:
			await self.on_stop_file(StopFileMessage(file_name=value['FileName']))

	@logger.catch()
	async def processes_reader(self):
		while True:
			if self.ws is None:
				await asyncio.sleep(0.5)
			else:
				try:
					await self.ws.send(json.dumps({
						"Log": self.reader_queue.get_nowait()
					}))
				except Empty:
					await asyncio.sleep(0.1)
			stopped_processes = []
			for name in self.processes.copy():
				if not self.processes[name].is_running:
					stopped_processes.append(name)
					self.processes.pop(name)
			if len(stopped_processes) > 0:
				information = self.read_files_info()
				for name in stopped_processes:
					information[name].is_running = False
				self.write_files_info(information)
				await self.send_update_files()

	async def _file_channel(self):
		while True:
			try:
				async with websockets.connect("ws://127.0.0.1:8030") as websocket:
					self.ws = websocket
					logger.debug("File channel started.")
					await websocket.send(LOCAL_SERVICE_KEYWORD)
					logger.debug("File channel identified as local service.")
					while True:
						data = await websocket.recv()
						try:
							decoded = json.loads(data)
						except json.JSONDecodeError:
							logger.warning(f"Received info event message with broken JSON data: {data}.")
							continue
						asyncio.create_task(self.route_event(decoded))
			except Exception as err:
				logger.error(err)

	def _run_control_thread(self):
		asyncio.set_event_loop(asyncio.new_event_loop())
		asyncio.get_event_loop().create_task(self.processes_reader())
		asyncio.get_event_loop().run_until_complete(self._file_channel())

	def _start_file_thread(self):
		logger.debug("Starting file channel thread.")
		self._file_channel_thread = Thread(target=self._run_control_thread, name="File channel thread")
		self._file_channel_thread.start()

	def start(self):
		logger.info("Starting robot.")
		self._start_file_thread()

if __name__ == '__main__':
	host = FileHost()
	host.start()