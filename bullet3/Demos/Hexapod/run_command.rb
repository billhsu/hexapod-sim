#!/usr/bin/env ruby
#Shipeng Xu
require 'socket'
filename = ARGV[0]
sock = TCPSocket.new('localhost', 5555)
line_num=0
text=File.open(filename).read
text.gsub!(/\r\n?/, "\n")
text.each_line do |line|
  line=line.gsub(/T.*$/, "")
  print "#{line_num += 1} #{line}"
  sock.write line
  sleep 0.4
end
sock.close