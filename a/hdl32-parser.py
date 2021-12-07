#!/usr/bin/env python

import struct
import dpkt
from itertools import izip_longest
import yaml
import pprint

def grouper(n, iterable, fillvalue=None):
    "Collect data into fixed-length chunks or blocks"
    # grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx
    args = [iter(iterable)] * n
    return izip_longest(fillvalue=fillvalue, *args)

def chunker(seq, size):
    return (seq[pos:pos + size] for pos in xrange(0, len(seq), size))

def printHex(byte, len):
	return "{0:0{1}x}".format(byte, len)

def calc_rotation(bytes):
  return (bytes[1]*255 + bytes[0])


y = open('db.yaml')
dataMap = yaml.load(y)
y.close()
#pp = pprint.PrettyPrinter(indent=4)
#pp.pprint(dataMap)

ls = dataMap['lasers']
for doc in ls:
	for key in doc.keys():
		print key, "->", doc[key]
		

	
print("opening file")
f = open('./unit_46_Monterey_subset.pcap', 'rb')
res = open("lidar.txt", "wb")

pcap = dpkt.pcap.Reader(f)

for ts, buf in pcap:
    eth = dpkt.ethernet.Ethernet(buf)
    ip = eth.data
    udp = ip.data
		
    if udp.dport == 2368 and len(udp.data) > 0:
		for i in range(1206):
			ba = bytearray(udp.data)
			firing_data = ba[0:1200]
			status_data = ba[1201:1206]
			for group in chunker(firing_data, 100):
				record = bytearray(group)
				start_id = record[0:2]
				hdata = "packet = "
				hdata += "startid="
				for byte in start_id:
					hdata += printHex(byte, 2) + " "
			
				rotational_position = record[2:4]
				hdata += "rotational_position=" + str(calc_rotation(rotational_position))
				lasers_data = record[4:100]
				hdata += "lasers[ = "
				dsr_num = 0
				for lasers in chunker(lasers_data, 3):
					hdata += "laser[ dsr#= " + str(dsr_num)
					for laser in lasers:
						hdata += printHex(laser, 2) + " "
					hdata += "]"
					dsr_num += 1
					
				res.write(hdata + "\n")	
res.close()	
f.close()
