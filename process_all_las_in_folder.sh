if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage : ./process_all_las_in_folder.sh <path/to/process_LAS_exec> <path/to/folder/containing/las/files>"
fi

for f in `find $2/ -name "*.las" -type f`; do
  echo "Processing $f file..."
  newfile=${f%.las}.ply
  echo "Saving $f in $newfile"
  $1 $f $newfile
done
