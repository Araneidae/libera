/* Extremely simple routine for reading a single word from the FPGA register
 * address space. */

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
    "Usage: %s [-d] [-c<count>] <register>\n"
    "   Reads hardware registers.\n"
    "Options:\n"
    "   -d  Return register value in decimal (default is hex)\n"
    "   -c: Read specified number of registers (default=1)\n";


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
}


static int read_registers(int base, int count, const char *format)
{
    int result = 2;     // Default on error.
    int mem = open("/dev/mem", O_RDONLY);
    if (mem < 0)
    {
        perror("Unable to open /dev/mem");
        return result;
    }
    if (lseek(mem, base, SEEK_SET) < 0)
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
            0, map_size, PROT_READ, MAP_SHARED, mem, first_page);
        if (register_base == MAP_FAILED)
            perror("Unable to map registers");
        else
        {
            print_registers(
                (const int *) (register_base + (base & OsPageMask)),
                count, format);
            result = 0;
            munmap((char *) register_base, map_size);
        }
    }
    
    close(mem);
    return result;
}


int main(int argc, char **argv)
{
    int count = 1;
    const char * format = "%08x";

    const char * argv0 = argv[0];
    int ch;
    while (ch = getopt(argc, argv, "hdc:"), ch != -1)
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
            default:
                return 1;
        }
    }
    argc -= optind;
    argv += optind;

    int offset;
    if (argc == 1  &&  read_int(argv[0], &offset, "register"))
    {
        return read_registers(offset, count, format);
        return 0;
    }
    else
    {
        printf("Run `%s -h` for usage\n", argv0);
        return 1;
    }
}
