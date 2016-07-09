#!/usr/bin/python

# Calculates the delta between a adjacent lines of a certain field in the input

from optparse import OptionParser
import sys

parser = OptionParser()
parser.add_option("-f", "--field",
                  dest="field",
                  help="the field to produce deltas for, indexed from 0",
                  metavar="INDEX",
                  default=0,
                  type="int")
parser.add_option("-o", "--output",
                  dest="output",
                  help="The field which the output delta field should be put before. The default is to output the delta after the field being considered.",
                  metavar="OUT",
                  default=None,
                  type="int")
parser.add_option("-q", "--quiet",
                  action="store_true", dest="quiet", default=False,
                  help="Only outputs the delta.")

parser.add_option("-m", "--multiply",
                  dest="factor",
                  help="Bit of a hack. Multiply the deltas by some factor.",
                  metavar="FACTOR",
                  default=1,
                  type="int")

(options, args) = parser.parse_args()

if options.output is None:
    options.output = options.field + 1


def out1():
    line.insert(options.output, str(val - prev))
    print " ".join(line)

def quietout():
    print val - prev

outputter = out1
if options.quiet:
    outputter = quietout

prev = None
for line in sys.stdin:
    line = line.strip().split()

    val = int(line[options.field])
    val *= options.factor

    if prev is not None:
        outputter()

    prev = val
    
