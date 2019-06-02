f=open("../.bashrc", "a+")
found = False
for line in f:
    if "$INIT_CMD" in line:
        found = True
        break

if not found:
	f.write("\nif [ ! -z \"$INIT_CMD\" ]; then\n\techo $INIT_CMD\n\texec $INIT_CMD\nfi")
f.close()