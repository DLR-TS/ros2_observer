# C++ Classes for Babeltrace 2 C API

A set of header only C++ classes which represent the opaque C structs of the
Babeltrace 2 C API.

> [!NOTE]
> These classes are incomplete. They do not cover the entirety of the
  Babeltrace 2 C API. For example, there is no class `Source` representing the
  struct `bt_self_component_source`. Similarly, the class `ClockClass` does not
  provide an equivalent function `getFrequency()` for the function
  `bt_clock_class_get_frequency()`.

## Requirements
Either a compiler that supports `c++17` and `libboost` version `1.56.0` or
newer for `boost::span` or a compiler that supports `c++20` which provides the
feature `std::span`.

## Usage
Include `<babeltrace2/babeltrace.h>` before including any of the header files.

## Example
See the sample sink component in the folder [examples][], which was adapted
from the [Babeltrace 2 documentation][sample-sink-component] to use the C++
classes.


[examples]: ../examples
[sample-sink-component]: https://babeltrace.org/docs/v2.1/libbabeltrace2/example-simple-sink-cmp-cls.html
