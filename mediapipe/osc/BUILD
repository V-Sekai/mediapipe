load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "osc",
    srcs = [
		"ip/posix/NetworkingUtils.cpp",
		"ip/posix/UdpSocket.cpp",
		"ip/win32/NetworkingUtils.cpp",
		"ip/win32/UdpSocket.cpp",
		"ip/IpEndpointName.cpp",
		"OscOutboundPacketStream.cpp",
		"OscPrintReceivedElements.cpp",
		"OscReceivedElements.cpp",
		"OscTypes.cpp",
    		],
    hdrs = [
		"ip/IpEndpointName.h",
		"ip/NetworkingUtils.h",
		"ip/PacketListener.h",
		"ip/TimerListener.h",
		"ip/UdpSocket.h",
		"MessageMappingOscPacketListener.h",
		"OscException.h",
		"OscHostEndianness.h",
		"OscOutboundPacketStream.h",
		"OscPacketListener.h",
		"OscPrintReceivedElements.h",
		"OscReceivedElements.h",
		"OscTypes.h",
    		],
    visibility = [
                   "//visibility:public"
                   ],
)
