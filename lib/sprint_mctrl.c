
#include <linux/module.h>
#include <linux/string.h>
#include <linux/termios.h>

size_t sprint_mctrl(char *buf, size_t size, int mctl)
{
	int l;

	buf[0] = 0;
	if (mctl & TIOCM_LE)
		strncat(buf, "LE ", size);
	if (mctl & TIOCM_DTR)
		strncat(buf, "DTR ", size);
	if (mctl & TIOCM_RTS)
		strncat(buf, "RTS ", size);
	if (mctl & TIOCM_ST)
		strncat(buf, "ST ", size);
	if (mctl & TIOCM_SR)
		strncat(buf, "SR ", size);
	if (mctl & TIOCM_CTS)
		strncat(buf, "CTS ", size);
	if (mctl & TIOCM_CAR)
		strncat(buf, "CAR ", size);
	if (mctl & TIOCM_RNG)
		strncat(buf, "RNG ", size);
	if (mctl & TIOCM_DSR)
		strncat(buf, "DSR ", size);
	if (mctl & TIOCM_OUT1)
		strncat(buf, "OUT1 ", size);
	if (mctl & TIOCM_OUT2)
		strncat(buf, "OUT2 ", size);
	if (mctl & TIOCM_LOOP)
		strncat(buf, "LOOP ", size);

	l = strlen(buf);
	if (l) buf[l-1] = 0; // drop the trailing space

	return l;
}

EXPORT_SYMBOL_GPL(sprint_mctrl);
