#!/usr/bin/perl -w

#
# Script to reprogram the CPCI
# $Id: cpci_reprogram.pl 3107 2007-12-10 21:45:09Z grg $
#

use strict;
use Cwd;
use Getopt::Long;

# Location of binaries
my $bindir = '/srv/nfnet/pktgen/oldkernelbin';
my $sbindir = '/srv/nfnet/pktgen/oldkernelbin';

# Location of bitfiles
my $bitfiledir = '/srv/nfnet/pktgen/oldkernelbin/bitfiles';

# Bitfiles
my $cpci = 'CPCI_2.1.bit';
my $cpci_reprogrammer = 'cpci_reprogrammer.bit';

# System binaries
my $lspci = '/sbin/lspci';
my $ifconfig = '/sbin/ifconfig';

# NetFPGA PCI device ID
my $NF2_device_id = "feed:0001";

my $output;
my $dir = getcwd;

# Parse the command line arguments
my $all = 0;   			# Program ALL devices
my $program_device = -1; 	# select which device to program default to 0
my $help = '';
my $cpci_bitfile = "$bitfiledir/$cpci";

unless ( GetOptions ( "all" => \$all,
		      "device=i" => \$program_device,
		      "help" => \$help,
		      "bit=s" => \$cpci_bitfile,
		     )
	 and ($help eq '')
       ) { usage(); exit 1 }

# Verify that the specified bitfile is valid
$cpci_bitfile = glob($cpci_bitfile);
if (! -f "$cpci_bitfile") {
	print "Error: cannot locate CPCI bitfile '$cpci_bitfile'\n\n";
	usage();
	exit(1);
}

# Work out the devices in the system 
my @device_id = get_lspci();

# Work out if the device ID is valid
if ($all && $program_device != -1) {
	print "Error: cannot specify a device ID at the same time as using the -all flag\n\n";
	usage();
	exit(1);
}

if ($program_device == -1) {
	$program_device = 0;
}

if ($program_device < 0 || $program_device > $#device_id) {
	print "Error: specified device ID ($program_device) is outside of valid range (0 .. " . ($#device_id + 1) . ")\n\n";
	usage();
	exit(1);
}

# Calculate the starting/ending IDs
my ($start_val, $end_val);

if ($all) {
	$start_val = 0;
	$end_val = $#device_id;
}
else {
	$start_val = $program_device;
	$end_val = $program_device;
}

# Bring the interfaces down
for (my $i = $start_val; $i <= $end_val; $i++) {
#	for (my $k = $i * 4; $k < ($i * 4) + 4; $k++) {
#  		`$ifconfig nf2c$k down`;
#	}
`$ifconfig nf2c$i down`;
}

# Reporgram the CPCI
for (my $i = $start_val; $i <= $end_val; $i++) {
	my $nf2c_base = $i;

	# Download the reprogrammer
	print("Loading the CPCI Reprogrammer on NetFPGA $i\n");
	$output = `$bindir/nf2_download -i nf2c$nf2c_base $bitfiledir/$cpci_reprogrammer 2>&1` 
		or die "Error downloading '$bitfiledir/$cpci_reprogrammer' to NetFPGA";
	
	if (!($output =~ m/successfully/i))
	{
	  print("\tDownload Failed\n");
	  exit(1);
	}

	# Dump the registers
	#print("$sbindir/dumpregs.sh $device_id[$i] 2>/dev/null\n");
	my $regs = `$sbindir/dumpregs.sh $device_id[$i] 2>/dev/null`;

	# Download the CPCI image
	print("Loading the CPCI on NetFPGA $i\n");

	$output = `$bindir/nf2_download -i nf2c$nf2c_base -c $cpci_bitfile 2>&1` 
		or die "Error downloading '$cpci_bitfile' to NetFPGA";

	if (!($output =~ m/Instructed CPCI reprogramming to start. Please reload PCI BARs./i))
	{
	  print("\tDownload Failed\n");
	  exit(1);
	}


	# Sleep for a while...
	`sleep 1.5`;

	# Restore the registers
	$output = `echo "$regs" | $sbindir/loadregs.sh $device_id[$i]`;

	print("CPCI on NetFPGA $i has been successfully reprogrammed\n");

}

# Bring the interfaces up
for (my $i = $start_val; $i <= $end_val; $i++) {
#	for (my $k = $i * 4; $k < ($i * 4) + 4; $k++) {
#  		`$ifconfig nf2c$k up`;
#	}
    `$ifconfig nf2c$i up`;
}

exit (0);

#########################################################
# usage
#   print usage information
sub usage {
  (my $cmd = $0) =~ s/.*\///;
  print <<"HERE1";
NAME
   $cmd - Dowload the latest CPCI design to the CPCI

SYNOPSIS
   $cmd 
        [--device <device number>] 
	[--bit <CPCI bit file>]
        [--all]

   $cmd --help  - show detailed help

HERE1

  return unless ($help);
  print <<"HERE";

DESCRIPTION

   This script downloads the latest CPCI image (located in the
   directory '$bitfiledir') to the NetFPGA board. This should 
   always be done on bootup to ensure the NetFPGA board is 
   running the latest firmware revision.
   

OPTIONS
   --device <device number>
     Specify which board to program in systems with multiple boards.
     This option is zero-indexed (ie. the first board in the system is
     referenced by board 0.)

   --bit <CPCI bit file>
     Reprogram the CPCI using the specified CPCI bit file.

   --all
     Program *all* boards in a system.


EXAMPLE

   To program all boards in the system:

   % $cmd -all

HERE
}


#########################################################
# get_lspci
#   run LSPCI and return all devices
sub get_lspci {
	my @device_list;
	my $output;
	
	# Run lspci
	$output = `$lspci -d $NF2_device_id 2>&1` 
		or die "Error running lspci";
  
	# Procoess the output
	foreach my $line (split(/\n/, $output)) {
		my @words = split(/\s+/, $line);

		# Search for the NetFPGA device ID
		if (index($line, $NF2_device_id) != -1)
		{
			push (@device_list, $words[0]);
		}
	}

	return @device_list;
}
