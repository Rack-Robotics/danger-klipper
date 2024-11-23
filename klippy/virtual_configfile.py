import configparser
from configfile import ConfigWrapper

error = configparser.Error


class sentinel:
    pass


class VirtualConfigFile(ConfigWrapper):
    def __init__(self, name, values, printer):
        self.name = name
        self.values = values
        self.printer = printer
        self.section = configparser.DEFAULTSECT
        self.fileconfig = configparser.RawConfigParser(
            strict=False, inline_comment_prefixes=(";", "#")
        )
    def get_name(self):
        return self.name

    def _get_wrapper(
        self,
        parser,
        option,
        default,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        note_valid=True,
    ):
        if option not in self.values:
            if default is not sentinel:
                return default
            raise error("Option '%s' is missing in values!" % option)
        try:
            v = parser(self.section, option, vars=self.values)
        except self.error as e:
            raise
        except:
            raise error("Unable to parse option '%s'" % option)
        if minval is not None and v < minval:
            raise error(
                "Option '%s' must have minimum of %s" % (option, minval)
            )
        if maxval is not None and v > maxval:
            raise error(
                "Option '%s' must have maximum of %s" % (option, maxval)
            )
        if above is not None and v <= above:
            raise error("Option '%s' must be above %s" % (option, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' must be below %s" % (option, below))
        return v

    def getsection(self, section):
        return self

    def has_section(self, section):
        return True

    def get_prefix_sections(self, prefix):
        return []

    def get_prefix_options(self, prefix):
        return []

    def deprecate(self, option, value=None):
        pass
