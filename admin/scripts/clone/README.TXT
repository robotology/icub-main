
Clone the robotcub.org repository to sf.net
===========================================

Configuration
-------------

Make a directory in "sources" for any CVS archive you wish to
copy from or to.  Currently there are four:
  sources/native-icub
  sources/native-icub-platform
  sources/sf-icub
  sources/sf-icub-platform
These should be a copy of the CVS directory used to check out the
desired sources.  In other words, they should contain a Root and 
Repository file.  For example, in sources/native-icub, we have:
  Repository -> "iCub"
  Root       -> "username@cvs.robotcub.org:/cvsroot/robotcub"
(change the user in the Root as appropriate).

Put the name of the repository you want to copy from in:
  source.txt
and the name of the repository you want to copy to in:
  target.txt

Operation
---------

If this is the first time to run, do:
  ./fresh.sh
This checks out the current state of the source and target repository.

Then run:
  ./prepare-actions.sh
This produces a long script apply.sh containing the necessary operations
on the target repository to make it match the source (in terms of content,
not revision history).

Check the generated apply.sh script for sanity, especially if this is
the first time to run.  Then do:
  ./apply.sh



- paulfitz
