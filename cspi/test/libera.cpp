// $Id: libera.cpp,v 1.10 2006/01/12 10:35:18 miha Exp $

// \file libera.cpp
// Simple utility for configuration and data acquisition.

/*
Libera
Copyright (C) 2004-2006 Instrumentation Technologies

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
or visit http://www.gnu.org

TAB = 4 spaces.
*/

#include <cerrno>
#include <cstdlib>
#include <cstdio>
#include <ctime>

#include <getopt.h>

#include <exception>
#include <stdexcept>

#include <limits>
#include <memory>
#include <string>
#include <iterator>
#include <algorithm>

#include <iosfwd>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>

#include "debug.h"
#include "cspi.h"

// Trigger timeout
#define TIMEOUT 30

// Helper macro to stringify the expanded argument
#define XSTR(s) STR(s)

// Stringification macro
#define STR(s) #s

#define CSPI_ERROR(what) cspi_error(what, __FUNCTION__, __LINE__)
#define SYSCALL_ERROR(what) syscall_error(what, __FUNCTION__, __LINE__)
#define PROGRAM_ERROR(what) program_error(what, __FUNCTION__, __LINE__)

//--------------------------------------------------------------------------

struct quote_type { const char *p; };

// ostream manipulator, i.e.: os << quote(str)
inline quote_type quote(const char *s)
{
	quote_type q;
	q.p = s;
	return q;
}

inline std::ostream& operator<<(std::ostream& os, quote_type q)
{
	return os << '\'' << q.p << '\'';
}

//--------------------------------------------------------------------------

// not used directly, serves as a base class only!
class program_error : public std::exception
{
public:
	explicit
	program_error(const char *what, const char *function, int line) throw()
	{
		std::ostringstream os;
		
		os << what
		   << " in function "
		   << quote(function)
		   << ": line " << line;

		what_str = os.str();
	}
	virtual ~program_error() throw() {}
	virtual const char* what() const throw() { return what_str.c_str(); }

protected:
	std::string what_str;
};

//--------------------------------------------------------------------------

// cspi error description, togehther with location in code
class cspi_error : public program_error
{
public:
	explicit
	cspi_error(int what, const char *function, int line) :
		program_error("CSPI error", function, line) {

		std::ostringstream os;
		os << ": " << cspi_strerror(what);

		if (CSPI_E_SYSTEM == what) {

			os << " -- "
			   << strerror(errno)
			   << " (" << errno << ")";
		}

		what_str += os.str();
	}
};

//--------------------------------------------------------------------------

// system call error description, togehther with location in code
class syscall_error : public program_error
{
public:
	explicit
	syscall_error(const char *what, const char *function, int line) :
		program_error("system call error", function, line) {

		std::ostringstream os;
		os << ": " << what
		   << " -- "
		   << strerror(errno)
		   << " (" << errno << ")";

		what_str += os.str();
	}
};

//--------------------------------------------------------------------------

// Note: this program does some performance critical formatted I/O.
//       With iostream classes, performance degraded by a factor of
//       10 to 15 in comparison to the C-style code. Thus, the later
//       is used to implement some sections.

// simple std::ostream_iterator counterpart for C streams (FILE*)
template <typename T>
class cstream_iterator : public std::iterator<std::output_iterator_tag, T>
{
public:
	cstream_iterator(FILE*& fp) : file(fp) {}

	cstream_iterator<T>& operator= (const T& val) {
		file << val;
		return *this;
	}

	// no-ops
	cstream_iterator& operator* () { return *this; }
	cstream_iterator& operator++ () { return *this; }
	cstream_iterator& operator++ (int) { return *this; }

protected:
	FILE*& file;
};

//--------------------------------------------------------------------------

void usage(const char* argv0)
{
	std::cout << "Usage: " << argv0 << " [OPTION]...\n"
	          << "       " << argv0 << " [OPTION]... SIZE\n"
	          << "       " << argv0 << " [OPTION]... TIME"
	          << std::endl;

	std::cout <<
	"\n"
	"Main operation mode:\n"
	"  -a, --acquire             acquire SIZE samples\n"
	"  -s, --set-environment     change environment variable(s)\n"
	"  -l, --list-environment    list all environment variables\n"
	"  -x, --set-time            set time to TIME\n"
	"\n"
	"Operation modifiers:\n"
	"  -b, --binary    output binary data\n"
	"\n"
	"Operation modifiers for --acquire:\n"
	"  -0, --using-dd            use DD history buffer as data-source\n"
	"  -1, --using-sa            use SA stream as data-source\n"
	"  -2, --using-pm            use PM buffer as data-source\n"
	"  -3, --using-adc           use ADC-rate buffer as data-source\n"
	"  -n [NUM], --loop[=NUM]    acquire in a loop NUM times,\n"
	"                            or infinitely if NUM omitted\n"
	"\n"
	"Operation modifiers for --using-dd or --using-pm:\n"
	"  -p, --with-timestamp    print a timestamp (first sample if\n"
	"                          --using-dd, last if --using-pm)\n"
	"  -r, --raw               acquire raw (I,Q) data\n"
	"\n"
	"Operation modifiers for --using-dd:\n"
	"  -d NUM, --decimation=NUM    set decimation to NUM\n"
	"  -o OFF, --offset=OFF        set offset in MT units\n"
	"  -t, --on-trigger            acquire data on trigger\n"
	"                              (default: current time)\n"
	"\n"
	"Informative output:\n"
	"  -h, --help       print this message, then exit\n"
	"  -v, --version    print version information, then exit\n"
	"\n"
	"SIZE    number of samples to acquire\n"
	"TIME    a colon separated MT:ST pair: [MT]:[YYYYMMDDhhmm.ss]\n"
	"To set MT or ST only, use 'MT:' or ':ST', respectively.\n"
	"\n"
	"Option --using-sa ignores --on-trigger.\n"
	"Options --using-adc and --set-time imply --on-trigger.\n"
	"\n"
	"  libera --set-environment < env.conf\n"
	"This will set Libera environment parameters listed in the\n"
	"configuration file. See sample configuration file env.conf\n"
	"included with the utility for more information.\n"
	"\n"
	"  libera --acquire --on-trigger --using-dd 1000 > some_file\n"
	"This will acquire 1000 data-on-demand samples on trigger into\n"
	"some_file. The operation will time out after 30 seconds if no\n"
	"trigger is received.\n"
	"\n"
	"  libera --acquire --on-trigger --using-dd --with-timestamp \\\n"
	"         --loop --raw 1000 > /dev/null\n"
	"This will acquire 1000 data-on-demand samples on trigger in\n"
	"a loop, discarding the data and printing a timestamp only on\n"
	"each iteration.\n"
	"\n"
	"  libera --set-time :200602091223.00\n"
	"This will set the system time (ST) at next trigger to Feb 09\n"
	"12:23:00 2006. Machine time (MT) will not change.\n";

	 std::cout << std::endl;
}

//--------------------------------------------------------------------------

void version(const char* argv0)
{
	std::cout << argv0 << ' ' << XSTR(RELEASE_VERSION)
	          << " (" __DATE__ ", " __TIME__ ") "
	          << std::endl
	          << std::endl;

	std::cout <<
		"Copyright 2005 Instrumentation Technologies.\n"
		"This is free software; see the source for copying conditions.\n"
		"There is NO warranty; not even for MERCHANTABILITY or FITNESS "
		"FOR A PARTICULAR PURPOSE.";

	std::cout << std::endl;
}

//--------------------------------------------------------------------------

// represents program configuration, initialized from command-line options
struct config
{
	config() :
		operation(unknown),
		mode(CSPI_MODE_UNKNOWN),
		atom_count(0),
		loop_count(1),
		mask(0)
		{}

	enum {unknown=0, acquire, setenv, listenv, settime};
	size_t operation;		// Main operation mode

	size_t mode;			// Acquisition mode
	struct dd_specific		// Specifics for DD acq. mode
	{
		dd_specific() : decimation(0), offset(0) {};

		size_t decimation;			// 1 or 64, 0=ignore
		unsigned long long offset;	// history buffer offset in MT units
	} dd;

	size_t atom_count;		// number of samples to retrieve
	size_t loop_count;		// number of iterations (repetitions)

	struct settime_specific
	{
		settime_specific() : mt(0), st(0) {}

		unsigned long long mt;		// machine time
		time_t st;					// system time (seconds since 1/1/1970)
	} time;

	enum {
		want_timestamp = 0x01,
		want_raw       = 0x02,
		want_trigger   = 0x04,
		want_binary    = 0x08,
		want_setmt     = 0x10,
		want_setst     = 0x20,
	};
	size_t mask;			// command-line switches (flags)
};

//--------------------------------------------------------------------------

// simple class to group functions to parse and verify command-line options
class option_parser
{
public:
	explicit option_parser(config &c) : cfg(c) {}
	~option_parser() {}

	int parse(int argc, char* const argv[]);
	const config& get_config() const { return cfg; }

protected:
	// verification functions; always return input arg
	size_t verify_operation(size_t op);
	size_t verify_mode(size_t M);
	size_t verify_decimation(size_t D);
	size_t verify_atomcount(size_t N);

	// assign MT and ST from command line argument
	void assign_time(const char* time);

private:
	config& cfg;
};

//--------------------------------------------------------------------------

int option_parser::parse(int argc, char* const argv[])
{
	const struct option longopts[] =
	{
		// Must set one of the following
		{"acquire",          no_argument, 0, 'a'},
		{"set-environment",  no_argument, 0, 's'},
		{"list-environment", no_argument, 0, 'l'},
		{"set-time",         no_argument, 0, 'x'},

		// Apply to acquire
		{"using-dd",         no_argument, 0, '0'},
		{"using-sa",         no_argument, 0, '1'},
		{"using-pm",         no_argument, 0, '2'},
		{"using-adc",        no_argument, 0, '3'},

		// Apply to acquire and using-??
		{"decimation",       required_argument, 0, 'd'},
		{"offset",           required_argument, 0, 'o'},
		{"with-timestamp",   no_argument,       0, 'p'},
		{"raw",              no_argument,       0, 'r'},
		{"loop",             optional_argument, 0, 'n'},
		{"on-trigger",       no_argument,       0, 't'},

		{"binary",           no_argument,       0, 'b'},
		{"help",             no_argument,       0, 'h'},
		{"version",          no_argument,       0, 'v'},
		{0,0,0,0},
	};

	// must match order prescribed by longopts[]
	const size_t modes[] = {CSPI_MODE_DD, CSPI_MODE_SA, CSPI_MODE_PM, CSPI_MODE_ADC};

	// must match chars prescribed by longopts[]
	const char optstring[] = "aslx0123d:o:prn::tbhv";

	int ch = -1;
	while ((ch = getopt_long(argc, argv, optstring, longopts, 0)) != -1)
	{
		switch (ch) {

			case 'a':
				cfg.operation = verify_operation(config::acquire);
				break;

			case 's':
				cfg.operation = verify_operation(config::setenv);
				break;

			case 'l':
				cfg.operation = verify_operation(config::listenv);
				break;

			case 'x':
				cfg.operation = verify_operation(config::settime);
				break;

			case '0':
			case '1':
			case '2':
			case '3':
				cfg.mode = verify_mode(modes[ch-'0']);
				break;

			case 'd':
				cfg.dd.decimation = verify_decimation( atol(optarg) );
				break;

			case 'o':
				cfg.dd.offset = atoll(optarg);
				break;

			case 'p':
				cfg.mask |= config::want_timestamp;
				break;

			case 'r':
				cfg.mask |= config::want_raw;
				break;

			case 'n':
				cfg.loop_count = optarg ? atol(optarg) : std::numeric_limits<size_t>::max();
				break;

			case 't':
				cfg.mask |= config::want_trigger;
				break;

			case 'b':
				cfg.mask |= config::want_binary;
				break;

			case 'h':
				usage(argv[0]);
				return EXIT_SUCCESS;

			case 'v':
				version(argv[0]);
				return EXIT_SUCCESS;

			default:
				return EXIT_FAILURE;
		}
	}

	if (config::unknown == cfg.operation) cfg.operation = config::acquire;
	if (config::acquire == cfg.operation) {

		if (optind == argc) throw std::runtime_error("Missing argument -- 'SIZE'");
		cfg.atom_count = verify_atomcount( atoi(argv[optind]) );
	}
	else if (config::settime == cfg.operation) {

		if (optind == argc) throw std::runtime_error("Missing argument -- 'TIME'");
		assign_time(argv[optind]);
	}

	return -1;
}

//--------------------------------------------------------------------------

size_t option_parser::verify_operation(size_t op)
{
	const char what[] = "You may not specify more than one '-aslx' option";

	if (cfg.operation != config::unknown && cfg.operation != op)
		throw std::runtime_error(what);

	return op;
}

//--------------------------------------------------------------------------

size_t option_parser::verify_mode(size_t M)
{
	const char what[] = "You may not specify more than one '-0123' option";

	if (cfg.mode != CSPI_MODE_UNKNOWN && cfg.mode != M)
		throw std::invalid_argument(what);

	return M;
}

//--------------------------------------------------------------------------

size_t option_parser::verify_decimation(size_t D)
{
	const char what[] = "Invalid argument -- '-d'";
	if (D != 1 && D != 64) throw std::invalid_argument(what);

	return D;
}

//--------------------------------------------------------------------------

size_t option_parser::verify_atomcount(size_t N)
{
	const char what[] = "Invalid argument -- 'SIZE'";
	const size_t maxsize = (65536 - 1) * 32;	// Max atoms per read

	if (N > maxsize) throw std::invalid_argument(what);

	return N;
}

//--------------------------------------------------------------------------

// assign MT and ST from a string formatted as [MT]:[YYYYMMDDhhmm.ss]
void option_parser::assign_time(const char* time)
{
	const char delim = ':';
	std::string s(time);

	size_t p = s.find(delim);
	if (std::string::npos == p) throw std::runtime_error("Invalid argument -- 'TIME'");

	std::string s2(s.substr(0, p-0));
	if (!s2.empty()) {

		cfg.time.mt = atoll(s2.c_str());
		cfg.mask |= config::want_setmt;
	}

	s2 = s.substr(p+1);
	if (!s2.empty()) {

		for (p=4; p < (s2.size()-3); ++p) if (p%3 == 1) s2.insert(p, 1, delim);

		struct tm t;
		if (!strptime(s2.c_str(), "%Y:%m:%d:%H:%M.%S", &t))
			throw std::runtime_error("Invalid argument -- 'TIME'");

		cfg.time.st = mktime(&t);
		if (-1 == cfg.time.st) throw SYSCALL_ERROR("mktime");

		cfg.mask |= config::want_setst;
	}
}

//--------------------------------------------------------------------------

// dump CSPI_ENVPARAMS members with descriptions
std::ostream& operator<<(std::ostream& os, const CSPI_ENVPARAMS& obj)
{
	const char* labels[] = {
		"TRIGmode",
		"Kx", "Ky",
		"Xoffset", "Yoffset", "Qoffset",
		"Xlow", "Xhigh", "Ylow", "Yhigh",
		"Switches",
		//"attenuators",
		0
	};

	const int *p = reinterpret_cast<const int*>(&obj);
	for (size_t i=0; labels[i]; ++i) {

		os << std::setw(12) << labels[i] << ": " << *p++ << std::endl;
	}

	os << std::setw(12) << "Attenuators" << ": ";
	const int *q = p + CSPI_MAXATTN - 1;

	std::copy(p, q, std::ostream_iterator<int>(std::cout,", "));
	return os << *q << std::endl;
}

//--------------------------------------------------------------------------

struct attn_adaptor
{
	explicit attn_adaptor(int* p) : pval(p) {}
	std::istream& extract(std::istream& is) {

		for (int n=CSPI_MAXATTN; is && n; --n) is >> *pval++;
		return is;
	}

protected:
	int *pval;
};

//--------------------------------------------------------------------------

class gain_adaptor : public attn_adaptor
{
public:
	explicit gain_adaptor(int* p) : attn_adaptor(p) {}
	std::istream& extract(std::istream& is) {

		std::string gain;
		if (!(is >> gain)) return is;

		std::ifstream file("gain.conf");
		if (!file) throw std::runtime_error("cannot open gain.conf");

		std::string val;
		while (file >> val) {

			if (val == gain) {
				attn_adaptor::extract(file);
				break;
			}
			file.ignore(std::numeric_limits<int>::max(), '\n');
		}

		if (!file) is.setstate(std::ios::failbit);
		return is;
	}
};

//--------------------------------------------------------------------------

// cannot use attn_adaptor& if used as a stream modifier
inline std::istream& operator>>(std::istream& is, attn_adaptor obj)
{
	return obj.extract(is);
}

// cannot use gain_adaptor& if used as a stream modifier
inline std::istream& operator>>(std::istream& is, gain_adaptor obj)
{
	return obj.extract(is);
}

//--------------------------------------------------------------------------

struct keyword
{
	const enum adaptor_type {plain, attn, gain} type;

	const char* key;
	const size_t mask;

	int& val;

	// required for std::find et al.
	bool operator==(const std::string& s) const { return s==key; }
};

//--------------------------------------------------------------------------

// read a (CSPI_ENVPARAMS, mask) pair from an istream
std::istream& operator>>(std::istream& is,
                         std::pair<CSPI_ENVPARAMS, size_t>& pair)
{
	// Note: input stream should consist of parameters of a form:
	// NAME VALUE
	// The stream is line-based - that is, each newline-terminated
	// line represents either a comment, or a parameter.
	// Parameter names are case sensitive.
	// Leading and trailing whitespace in parameter names and values
	// is irrelevant and discarded.
	// Any  line beginning with a hash ('#') character is ignored,
	// as are lines containing only whitespace.
	// 'The VALUE for Attenuators' field consists of CSPI_MAXATTN
	// whitespace separated values.

	#define KEY(A,K,M,V) {A, K, CSPI_ENV_##M, pair.first.V}
	keyword map[] = {

		KEY(keyword::plain, "TRIGmode", TRIGMODE, trig_mode),

		KEY(keyword::plain, "Kx", KX, Kx),
		KEY(keyword::plain, "Ky", KY, Ky),

		KEY(keyword::plain, "Xoffset", XOFFSET, Xoffset),
		KEY(keyword::plain, "Yoffset", YOFFSET, Yoffset),
		KEY(keyword::plain, "Qoffset", QOFFSET, Qoffset),

		KEY(keyword::plain, "Xlow",  XLOW,  Xlow),
		KEY(keyword::plain, "Xhigh", XHIGH, Xhigh),
		KEY(keyword::plain, "Ylow",  YLOW,  Ylow),
		KEY(keyword::plain, "Yhigh", YHIGH, Yhigh),

		KEY(keyword::plain, "Switches", SWITCH, switches),

		KEY(keyword::attn, "Attenuators",  ATTN, attn[0]),
		KEY(keyword::gain, "Gain",         ATTN, attn[0]),
	};

	keyword *const beg = map;
	keyword *const end = beg + sizeof(map)/sizeof(keyword);

	std::string line, key;
	while (std::getline(is, line)) {

		std::istringstream iss(line);
		iss >> key;

		// skip empty lines and comments
		if (iss && key[0] != '#') {

			keyword *p = std::find(beg, end, key);
			if (end == p) {

				std::string what("Invalid keyword: ");
				what += key;

				throw std::invalid_argument(what);
			}

			switch (p->type) {

				case keyword::attn:
					iss >> attn_adaptor(&p->val);
					break;

				case keyword::gain:
					iss >> gain_adaptor(&p->val);
					break;

				default:
					iss >> p->val;
			}
			pair.second |= p->mask;

			if (!iss) {

				std::string what("Invalid or missing value: ");
				what += key;

				throw std::invalid_argument(what);
			}
		}
	}
	return is;
}

//--------------------------------------------------------------------------

// represents cspi handle
template <typename cspiH>
struct cspi_handle
{
	explicit cspi_handle(cspiH h=0) : handle(h) {}
	virtual ~cspi_handle() {}

	operator cspiH() { return handle; }
	cspiH handle;

private:
	cspi_handle(cspi_handle<cspiH>&) {}
};

//--------------------------------------------------------------------------

// represents cspi environment handle
struct cspihenv : public cspi_handle<CSPIHENV>
{
	// uses "construction is initialization" pattern
	explicit cspihenv(bool as_su=0) : cspi_handle<CSPIHENV>() {

		CSPI_LIBPARAMS lib = {1,1};
		if (as_su) cspi_setlibparam(&lib, CSPI_LIB_SUPERUSER);
	
		int rc = cspi_allochandle(CSPI_HANDLE_ENV, 0, &handle);
		if (CSPI_OK != rc) throw CSPI_ERROR(rc);
	};

	~cspihenv() {

		int rc = cspi_freehandle(CSPI_HANDLE_ENV, handle);
		if (CSPI_OK != rc) throw CSPI_ERROR(rc);
	}
};

//--------------------------------------------------------------------------

// represents cspi connection handle
struct cspihcon : public cspi_handle<CSPIHCON>
{
	// uses "construction is initialization" pattern
	explicit cspihcon(CSPIHENV henv) : cspi_handle<CSPIHCON>() {

		int rc = cspi_allochandle(CSPI_HANDLE_CON, henv, &handle);
		if (CSPI_OK != rc) throw CSPI_ERROR(rc);
	};

	~cspihcon() {

		int rc = cspi_freehandle(CSPI_HANDLE_CON, handle);
		if (CSPI_OK != rc) throw CSPI_ERROR(rc);
	}
};

//--------------------------------------------------------------------------

// simple base class to implement different read operations
struct reader_base
{
	reader_base() {}
	virtual ~reader_base() {}

	virtual int read(CSPIHCON h, void *dest, size_t count, size_t *nread) = 0;
};

//--------------------------------------------------------------------------

// reader to retrieve raw (I,Q) data
struct raw_reader : public reader_base
{
	int read(CSPIHCON h, void *dest, size_t count, size_t *nread) {
		return cspi_read_ex(h, dest, count, nread, 0);
	}
};

//--------------------------------------------------------------------------

// default reader to retrieve calculated data (positions)
struct def_reader : public reader_base
{
	int read(CSPIHCON h, void *dest, size_t count, size_t *nread) {
		return cspi_read(h, dest, count, nread);
	}
};

//--------------------------------------------------------------------------

// base class for different operations (tasks) in this program
class task
{
public:
	explicit task(config& c) : cfg(c) {}
	virtual ~task() {}
	 
	virtual int run() = 0;

protected:
	config& cfg;
};

//--------------------------------------------------------------------------

// synchronize time
class settime_task : public task
{
public:
	explicit settime_task(config& c) : task(c), henv(1) {}
	int run();

protected:
	cspihenv henv;
};

//--------------------------------------------------------------------------

// set env. variables
class setenv_task : public task
{
public:
	explicit setenv_task(config& c) : task(c), henv(1) {}
	int run();

protected:
	cspihenv henv;
};

//--------------------------------------------------------------------------

// list env. variables
class listenv_task : public task
{
public:
	explicit listenv_task(config& c) : task(c), henv() {}
	int run();

protected:
	cspihenv henv;
};

//--------------------------------------------------------------------------

// data acquisition task
class acq_task : public task
{
public:
	explicit acq_task(config& c, CSPI_BITMASK m=0) : task(c), henv(), hcon(henv) {

		pre_connect(m);

		int rc = cspi_connect(hcon);
		if (CSPI_OK != rc) throw CSPI_ERROR(rc);
	}

	~acq_task() {

		int rc = cspi_disconnect(hcon);
		if (CSPI_OK != rc) throw CSPI_ERROR(rc);
	}

protected:
	cspihenv henv;
	cspihcon hcon;

	void pre_connect(CSPI_BITMASK event_mask);
};

//--------------------------------------------------------------------------

// implements acquisition of streaming data
class streaming_acq_task : public acq_task
{
public:
	explicit streaming_acq_task(config& c) : acq_task(c) {}
	~streaming_acq_task() {}

	typedef CSPI_SA_ATOM atom_type;

protected:
	int run();
	int acquire(atom_type& a);
};

typedef streaming_acq_task sa_task;

//--------------------------------------------------------------------------

// fwd decl -- class factory
struct factory
{
	static task* new_task(config&);
	static acq_task* new_acq_task(config&);
	static reader_base* new_reader(config&);
};

//--------------------------------------------------------------------------

// implements acquisition of non-streaming data
template <typename cspiT>
class nonstreaming_acq_task : public acq_task
{
public:
	// CSPI_EVENT_CFG needed to detect Kx,Ky and offset changes in CSPI
	explicit nonstreaming_acq_task(config& c) : acq_task(c, CSPI_EVENT_CFG),
		reader(factory::new_reader(c)) {}

	~nonstreaming_acq_task() {}
	typedef cspiT atom_type;

protected:
	const std::auto_ptr<reader_base> reader;

	int run();
	virtual int acquire(atom_type *p, size_t size) = 0;

private:
	int wait_trigger();
	void write_timestamp();
};

//--------------------------------------------------------------------------

// specialization for data-on-demand
class dd_task : public nonstreaming_acq_task<CSPI_DD_ATOM>
{
public:
	explicit dd_task(config& c) : nonstreaming_acq_task<CSPI_DD_ATOM>(c) {

		if (cfg.dd.decimation) {
	
			CSPI_CONPARAMS_DD p;
			p.dec = cfg.dd.decimation;

			int rc = cspi_setconparam(hcon, (CSPI_CONPARAMS *)&p, CSPI_CON_DEC);
			if (CSPI_OK != rc) throw CSPI_ERROR(rc);
		}
	}
	~dd_task() {}

protected:
	int acquire(atom_type *p, size_t size);
};

//--------------------------------------------------------------------------

// specialization for post-mortem data
class pm_task : public nonstreaming_acq_task<CSPI_DD_ATOM>
{
public:
	explicit pm_task(config& c) : nonstreaming_acq_task<CSPI_DD_ATOM>(c) {}
	~pm_task() {}

protected:
	int acquire(atom_type *p, size_t size);
};

//--------------------------------------------------------------------------

// specialization for ADC-rate data
class adc_task : public nonstreaming_acq_task<CSPI_ADC_ATOM>
{
public:
	explicit adc_task(config& c) : nonstreaming_acq_task<CSPI_ADC_ATOM>(c) {}
	~adc_task() {}

protected:
	int acquire(atom_type *p, size_t size);
};

//--------------------------------------------------------------------------

int settime_task::run()
{
	CSPI_TIMESTAMP ts;
	CSPI_BITMASK mask = 0;

	if (cfg.mask & config::want_setmt) {

		mask |= CSPI_TIME_MT;
		ts.mt = cfg.time.mt;
	}
	if (cfg.mask & config::want_setst) {

		mask |= CSPI_TIME_ST;
		ts.st.tv_sec = cfg.time.st;
		ts.st.tv_nsec = 0;
	}

	CSPI_ENVPARAMS ep;
	ep.trig_mode = CSPI_TRIGMODE_SET;

	int rc = cspi_setenvparam(henv, &ep, CSPI_ENV_TRIGMODE);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	rc = cspi_settime(henv, &ts, mask);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	return 0;
}
//--------------------------------------------------------------------------

int setenv_task::run()
{
	std::pair<CSPI_ENVPARAMS,size_t> p;

	int rc;
	if (cfg.mask & config::want_binary) {

		rc = fread(&p.first, sizeof(p.first), 1, stdin);
		if (ferror(stdin)) throw SYSCALL_ERROR("fread");
	}
	else std::cin >> p;

	rc = cspi_setenvparam(henv, &p.first, p.second);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	return 0;
}

//--------------------------------------------------------------------------

int listenv_task::run()
{
	CSPI_ENVPARAMS params;
	const CSPI_BITMASK mask = ~0;

	int rc = cspi_getenvparam(henv, &params, mask);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	if (cfg.mask & config::want_binary) {

		rc = fwrite(&params, sizeof(params), 1, stdout);
		if (rc != 1) throw SYSCALL_ERROR("fwrite");
	}
	else std::cout << params << std::endl;

	return 0;
}

//--------------------------------------------------------------------------

volatile size_t _event_id = 0;

int event_callback(CSPI_EVENT *p)
{
	_event_id = p->hdr.id;
	return 0;
}

//--------------------------------------------------------------------------

void acq_task::pre_connect(CSPI_BITMASK event_mask)
{
	if (cfg.mask & config::want_trigger) event_mask |= CSPI_EVENT_TRIGGET;

	CSPI_CONPARAMS p;

	p.mode = cfg.mode;
	p.handler = event_callback;
	p.event_mask = event_mask;

	CSPI_BITMASK param_mask = CSPI_CON_MODE;
	if (event_mask) param_mask |= (CSPI_CON_HANDLER|CSPI_CON_EVENTMASK);

	int rc = cspi_setconparam(hcon, &p, param_mask);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);
}

//--------------------------------------------------------------------------

// fwd decl
FILE*& operator<<(FILE*&, const streaming_acq_task::atom_type&);

//--------------------------------------------------------------------------

int streaming_acq_task::run()
{
	if (!cfg.atom_count) return 0;

	atom_type atom;

	size_t count = cfg.loop_count * cfg.atom_count;
	while (count--) {

		acquire(atom);
		if (cfg.mask & config::want_binary) {

			VERIFY(1 == fwrite(&atom, sizeof(atom), 1, stdout));
		}
		else stdout << atom;
	}

	return 0;
}

//--------------------------------------------------------------------------

inline int streaming_acq_task::acquire(atom_type& atom)
{
	int rc = cspi_get(hcon, &atom);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	return 0;
}

//--------------------------------------------------------------------------

template <typename cspiT>
int nonstreaming_acq_task<cspiT>::run()
{
	if (!cfg.atom_count) return 0;
	const size_t size = cfg.atom_count;

	cspiT *p = new cspiT[size];	// cannot use std::auto_ptr<>, see catch!
	try {

		size_t count = cfg.loop_count;
		while (count--) {

			if (cfg.mask & config::want_trigger) wait_trigger();
			acquire(p, size);
	
			if (cfg.mask & config::want_timestamp) write_timestamp();
			if (cfg.mask & config::want_binary) {

				VERIFY(size == fwrite(p, sizeof(cspiT), size, stdout));
			}
			else std::copy(p, p+size, cstream_iterator<cspiT>(stdout));
		}
	}
	catch(...) {

		delete[] p;
		throw;
	}

	delete[] p;
	return 0;
}

//--------------------------------------------------------------------------

template <typename cspiT>
int nonstreaming_acq_task<cspiT>::wait_trigger()
{
	size_t nleft = TIMEOUT;
	do {

		nleft = sleep(nleft);
	}
	while(nleft && _event_id != CSPI_EVENT_TRIGGET);

	if (0 == nleft) throw std::runtime_error("Trigger timeout");
	return nleft;
}

//--------------------------------------------------------------------------

//fwd decl
FILE*& operator<<(FILE*& os, const CSPI_TIMESTAMP& ts);

//--------------------------------------------------------------------------

template <typename cspiT>
void nonstreaming_acq_task<cspiT>::write_timestamp()
{
	CSPI_TIMESTAMP ts;

	int rc = cspi_gettimestamp(hcon, &ts);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	// config::want_binary does not apply to timestamp
	stderr << ts;
}

//--------------------------------------------------------------------------

int dd_task::acquire(atom_type *p, size_t size)
{
	int rc;

	static bool first_time = true;
	if (first_time) {

		first_time = false;
		rc = cfg.mask & config::want_trigger ? CSPI_SEEK_TR : 0;

		rc = cspi_seek(hcon, &cfg.dd.offset, rc);
		if (CSPI_OK != rc) throw CSPI_ERROR(rc);
	}

	size_t nread;
	rc = reader->read(hcon, p, size, &nread);
	if (CSPI_OK != rc) {

		if (CSPI_W_INCOMPLETE != rc) throw CSPI_ERROR(rc);

		std::stringstream os;
		std::cerr << "WARNING: "
		          << cspi_strerror(rc)
		          << ": "
		          << nread << "/" << size
		          << std::endl;

		atom_type empty = {0,0,0,0,0,0,0,0};
		std::fill(p + nread, p + size, empty);
	}

	return nread;
}

//--------------------------------------------------------------------------

int pm_task::acquire(atom_type *p, size_t size)
{
	size_t nread;

	int rc = reader->read(hcon, p, size, &nread);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	return nread;
}

//--------------------------------------------------------------------------

int adc_task::acquire(atom_type *p, size_t size)
{
	size_t nread;

	int rc = reader->read(hcon, p, size, &nread);
	if (CSPI_OK != rc) throw CSPI_ERROR(rc);

	return nread;
}

//--------------------------------------------------------------------------

inline
FILE*& operator<<(FILE*& os, const CSPI_TIMESTAMP& ts)
{
	char str[64];
	strftime(str, sizeof(str), "%F %T", gmtime(&ts.st.tv_sec));

	fprintf(os, "MT: %llu, ST: %s %06ld.%ld UTC\n",
	        ts.mt,
		    str,
		    ts.st.tv_nsec / 1000,
		    ts.st.tv_nsec % 1000 / 10);

	return os;
}

//--------------------------------------------------------------------------

inline
FILE*& operator<<(FILE*& os, const streaming_acq_task::atom_type& obj)
{
	fprintf(os, "%d %d %d %d %d %d %d %d %d %d\n",
			obj.Va,
			obj.Vb,
			obj.Vc,
			obj.Vd,
			obj.X,
			obj.Y,
			obj.Q,
			obj.Sum,
			obj.Cx,
			obj.Cy
	);
	return os;
}

//--------------------------------------------------------------------------

inline
FILE*& operator<<(FILE*& os, const dd_task::atom_type& obj)
{
	fprintf(os, "%d %d %d %d %d %d %d %d\n",
			obj.Va,
			obj.Vb,
			obj.Vc,
			obj.Vd,
			obj.X,
			obj.Y,
			obj.Q,
			obj.Sum
	);
	return os;
}

//--------------------------------------------------------------------------

inline
FILE*& operator<<(FILE*& os, const adc_task::atom_type& obj)
{
	fprintf(os, "%d %d %d %d\n", obj.chA, obj.chB, obj.chC, obj.chD);
	return os;
}

//--------------------------------------------------------------------------

task* factory::new_task(config& cfg)
{
	switch (cfg.operation) {

		case config::acquire:
			return new_acq_task(cfg);

		case config::setenv:
			return new setenv_task(cfg);

		case config::listenv:
			return new listenv_task(cfg);

		case config::settime:
			return new settime_task(cfg);
	}

	// Should never happen in non-debug code!
	throw std::invalid_argument("Invalid operation code");
	return 0;
}

//--------------------------------------------------------------------------

acq_task* factory::new_acq_task(config& cfg)
{
	switch (cfg.mode) {

		case CSPI_MODE_DD:
			return new dd_task(cfg);

		case CSPI_MODE_SA:
			cfg.mask &= ~config::want_trigger;
			return new sa_task(cfg);

		case CSPI_MODE_ADC:
			cfg.mask |= config::want_trigger;
			return new adc_task(cfg);

		case CSPI_MODE_PM:
			cfg.mask &= ~config::want_trigger;
			return new pm_task(cfg);
	}
	// Should never happen in non-debug code!
	throw std::invalid_argument("Invalid mode code");
	return 0;
}

//--------------------------------------------------------------------------

reader_base* factory::new_reader(config& cfg)
{
	switch (cfg.mode) {

		case CSPI_MODE_DD:
		case CSPI_MODE_PM:

			if (cfg.mask & config::want_raw) return new raw_reader;
			return new def_reader;

		case CSPI_MODE_ADC:
			return new def_reader;
	}
	// Should never happen in non-debug code!
	throw std::invalid_argument("Invalid mode code");
	return 0;
}

//--------------------------------------------------------------------------

int main(int argc, char* const argv[])
{
	// disable synchronization of C++ and C streams
	std::ios::sync_with_stdio(false);

	try {
		config cfg;
		option_parser parser(cfg);

		int rc;
		if ( (rc=parser.parse(argc, argv)) != -1 ) return rc;

		std::auto_ptr<task> ptask(factory::new_task(cfg));
		ptask->run();
	}
	catch (std::exception &e) {

		std::cerr << argv[0] << ": " << e.what() << std::endl;
	}
	return EXIT_SUCCESS;
}
