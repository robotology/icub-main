#!/usr/bin/perl -w

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

my $fname = shift @ARGV;
my $line_num=0;
my $max_lines=60;
my %good_licenses= ( # a list of "good" licenses
		     "Released under the terms of the GNU GPL v2.0"=> "GPL20",
             "under the terms of the GNU General Public License, version 2"=>"GPL20",
             "Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:"=>"BSD",

);


$fname=~s|\/\.\/|\/|;  # remove /./ pattern in directory names
$fname=~s|\/\/|\/|;    # remove // pattern in directory names

open(FILE, $fname) || 
    die "Can't open $fname";

my $author="unknown";
my $copyright="unknown";
my $license="unknown";

# Perform basic conversions to fix common problems
sub fix_names {
        my $names=$_[0];
        $names=~s/Alex Bernardino/Alexandre Bernardino/;
        $names=~s/\,?\s?European Commission FP6 Project IST(\-004370)?//;
        $names=~s/Robotcub/RobotCub/;
        $names=~s/^The\s|^\-//;

        $names=~s/\$YOUR_NAME/unknown/;

        return $names;
}


## Detect authors, copyright and licenses
while(!eof(FILE) && defined (my $line=<FILE>) && $line_num<$max_lines) {
    $line_num++;

    # next unless $line=~m/Copyright|Author|Copypolicy/i;
    chomp $line;
    # print "$line_num: $line\n";

    #### search for known good licenses
    while ( ($key, $value) = each %good_licenses) {
            
            if ($line=~m/$key/i) {
                #print "Match: $line_num: $line $value\n";
                $license=$value;
        }
    }

    # now skip some tricky sentences that contain the word "author"
    # in licensing context
    next if $line=~m/The name of the author may not be used/i;
    next if $line=~m/BE LIABLE FOR ANY/i;
    next if $line=~m/\`*AS IS\'* AND ANY EXPRESS/i;

    if ($line=~m/authors?:?\s*/i && $author eq "unknown"){
        $author=$';
        $author=~s/\.?\s*$//;  #remove trailing . and eof ...
        
        $author=fix_names($author);
    }

    if ($line=~m/copyright\s?(\s*:?\s*)?(\(C\))?\s*(((<?\d*>?)|(\d*))(\-?|\,(\d*)|(\d*))*)*\s*\,?\s*(.*)$/i && $copyright eq "unknown"){
        $line=$10;

        # now knock all the trailing 2010-2010 etc..
        $line=~s/(\s*\,?\s*(\,?|\-?|\s?(\d*))*)\$//i;
        $line=~s/\.?\s*$//;  #remove trailing . and eof ...
        #print"--> $`<$&>$' $line\n";
        $copyright=$line;
        $copyright=fix_names($copyright);
    }
}

print "File:$fname\n";
print "Copyright:$copyright\n";
print "Author:$author\n";
print "License:$license\n";
