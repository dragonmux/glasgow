
from types import coroutine
from typing import Optional
from amaranth import Signal
from amaranth.sim import Settle

from ... import GlasgowAppletTestCase, synthesis_test, applet_simulation_test
from ....gateware.pads import Pads

from . import JTAGPDIApplet, JTAGPDIInterface, Header

class PDIAppletTestCase(GlasgowAppletTestCase, applet = JTAGPDIApplet):
	@synthesis_test
	def test_build(self):
		from os import remove

		self.assertBuilds()
		self.assertBuilds(args = ["--raw-vcd", "test.file"])
		remove('test.file')

	def setup_simulation(self):
		self.build_simulated_applet()

	@staticmethod
	def step(cycles : int):
		for _ in range(cycles):
			yield

	@staticmethod
	def settle(cycles : int = 1):
		for _ in range(cycles):
			yield Settle()
			yield
		yield Settle()

	@staticmethod
	def wait_until_high(signal : Signal, *, timeout : Optional[int] = None):
		elapsedCycles = 0
		while not (yield signal):
			yield
			elapsedCycles += 1
			if timeout and elapsedCycles > timeout:
				raise RuntimeError(f'Timeout waiting for \'{signal.name}\' to go high')

	@staticmethod
	def wait_until_low(signal : Signal, *, timeout : Optional[int] = None):
		elapsedCycles = 0
		while (yield signal):
			yield
			elapsedCycles += 1
			if timeout and elapsedCycles > timeout:
				raise RuntimeError(f'Timeout waiting for \'{signal.name}\' to go low')

	@coroutine
	def write_id_code(self, pads : Pads, idcode : int):
		# Having issued the ID Code request, wait for the gateware to complete the IDLE phase
		yield from self.wait_until_high(pads.tms_t.o)
		# Now wait for it to clock through Capture-IR
		yield from self.wait_until_low(pads.tms_t.o)
		# Wait for Shift-IR to complete
		yield from self.wait_until_high(pads.tms_t.o)
		# Now wait for it to clock through Capture-DR
		yield from self.wait_until_low(pads.tms_t.o)
		# Re-align ourselves to the end of the phase on the falling clock edge
		yield from self.wait_until_high(pads.tck_t.o)
		yield from self.wait_until_low(pads.tck_t.o)
		yield from self.settle(2)

		# Loop through each bit in the ID code, putting it out on tdo_t (gateware TDI)
		for bit in range(32):
			yield pads.tdo_t.i.eq((idcode >> bit) & 1)
			yield Settle()
			yield from self.wait_until_low(pads.tck_t.o)
			yield from self.wait_until_high(pads.tck_t.o)

	@applet_simulation_test('setup_simulation', args = ['--interactive'])
	async def test_interactive_reset(self):
		from . import meta
		self.iface : JTAGPDIInterface = await self.run_simulated_applet()
		iface = self.iface.lower
		pads : Pads = self.applet.pads

		# Try to reset the scan chain
		await iface.write([Header.Reset])
		self.assertEqual(meta.to_int8(await iface.read(length = 1)), 1)

		# Now try to request the ID code from the device
		await iface.write([Header.IDCode])
		await self.write_id_code(pads, 0x6984203f)
		idcode = meta.decode_device_id_code(await iface.read(length = 4))
		self.assertEqual(idcode[0], '0x6984203f')
		self.assertEqual(idcode[1], 'Atmel')
		self.assertEqual(idcode[2], 6)
		self.assertEqual(idcode[3], 'ATXMega256A3U')
