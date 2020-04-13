gdb-multiarch 	-ex "python gdb.COMPLETE_EXPRESSION = gdb.COMPLETE_SYMBOL" \
		-ex "add-auto-load-safe-path scripts/gdb/vmlinux-gdb.py" \
		-ex "file vmlinux" \
		-ex "target remote :1234" 
