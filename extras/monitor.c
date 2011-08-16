/* This file is part of the Libera EPICS Driver,
 * Copyright (C) 2009-2011  Michael Abbott, Diamond Light Source Ltd.
 *
 * The Libera EPICS Driver is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * The Libera EPICS Driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:
 *      Dr. Michael Abbott,
 *      Diamond Light Source Ltd,
 *      Diamond House,
 *      Chilton,
 *      Didcot,
 *      Oxfordshire,
 *      OX11 0DE
 *      michael.abbott@diamond.ac.uk
 */

/* Extremely simple routine for reading 32-bit registers from IO space. */

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>


static const char * USAGE =
    "Usage: %s [-d] [-c<count>] [-W] <register> [<value>]\n"
    "   Reads hardware registers.\n"
    "Options:\n"
    "   -d  Return register value in decimal (default is hex)\n"
    "   -c: Read specified number of registers (default=1)\n"
    "   -W  Write value to register instead of reading\n";


static bool read_int(char *str, int *value, const char *name)
{
    char *end;
    *value = strtol(str, &end, 0);
    if (str < end  &&  *end == '\0')
        return true;
    else
    {
        fprintf(stderr, "\"%s\" not a number for %s\n", str, name);
        return false;
    }
}


static void print_registers(
    const int *registers, int count, const char *format)
{
    for (int i = 0; i < count; i ++)
    {
        printf(format, registers[i]);
        if (i % 8 == 7)
            printf("\n");
        else
            printf(" ");
    }
    if (count % 8 != 0)
        printf("\n");
}


static int * map_memory(int base, int count)
{
    int mem = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem < 0)
        perror("Unable to open /dev/mem");
    else if (lseek(mem, base, SEEK_SET) < 0)
        perror("Unable to seek to register offset");
    else
    {
        /* Alas, we can't just read from /dev/mem, instead have to memory map
         * it instead.  Don't really know why not. */
        int OsPageSize = getpagesize();
        int OsPageMask = OsPageSize - 1;
        /* Tricky calculation of number of pages to map. */
        int first_page = base & ~OsPageMask;
        int last_page = (base + sizeof(int) * count - 1) & ~OsPageMask;
        int map_size = last_page - first_page + OsPageSize;
        const char * register_base = (const char *) mmap(
            0, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, mem, first_page);
        if (register_base == MAP_FAILED)
            perror("Unable to map registers");
        else
            /* We don't bother to tidy up after ourselves, we haven't long to
             * live! */
            return (int *) (register_base + (base & OsPageMask));
    }

    /* If we get this far, we failed. */
    return NULL;
}


static int read_registers(int base, int count, const char *format)
{
    const int * registers = map_memory(base, count);
    if (registers == NULL)
        return 2;
    else
    {
        print_registers(registers, count, format);
        return 0;
    }
}


static int write_register(int base, int value)
{
    int * registers = map_memory(base, 1);
    if (registers == NULL)
        return 2;
    else
    {
        *registers = value;
        return 0;
    }
}


int main(int argc, char **argv)
{
    int count = 1;
    const char * format = "%08x";
    bool write = false;

    const char * argv0 = argv[0];
    int ch;
    while (ch = getopt(argc, argv, "hdc:W"), ch != -1)
    {
        switch (ch)
        {
            case 'h':
                printf(USAGE, argv0);
                return 0;
            case 'd':
                format = "%d";
                break;
            case 'c':
                if (!read_int(optarg, &count, "-c"))
                    return 1;
                break;
            case 'W':
                write = true;
                break;
            default:
                return 1;
        }
    }
    argc -= optind;
    argv += optind;

    int offset;
    if (argc == (write ? 2 : 1)  &&  read_int(argv[0], &offset, "register"))
    {
        if (write)
        {
            int value;
            if (read_int(argv[1], &value, "value"))
                return write_register(offset, value);
            else
                return 2;
        }
        else
            return read_registers(offset, count, format);
    }
    else
    {
        printf("Run `%s -h` for usage\n", argv0);
        return 1;
    }
}
