for /R ..\ %%G in (*.c, *.h) do dos2unix.exe "%%G"
for /R ..\ %%G in (*.c, *.h) do dos2unix_64.exe "%%G"