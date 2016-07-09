#!/usr/bin/python

import sys

args = sys.argv[1:]
if len(args) != 1:
    print >> sys.stderr, """Usage: avg_error.py VALUE
where VALUE is the correct, expected value that the errors are computed from."""
    exit(1)

correct = int(args[0])

num = 0
err_total = 0
total = 0

values = []

for line in sys.stdin:
    value = int(line.strip()) * 8
    values.append(value)
    err_total += abs(value - correct)
    total += value
    num += 1

avg_err = float(err_total) / num
mean = float(total) / num

var = 0
for v in values:
    var += (v - mean)**2

var = var / num

if num == 0:
    print '-'
else:
    print "mean %.2f var %.2f avgerr %.2f" % (mean, var, avg_err)
