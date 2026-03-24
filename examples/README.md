# Examples
Currently this folder only contains an adapted version of
the [sample sink component][sample-sink-component] from the
Babeltrace 2 documentation, `epitome.cpp`.

## License
In general the files in this directory are licensed under the
[Eclipse Public License - v 2.0][epl-2.0]. An exception is the file
`epitome.cpp`, which is licensed under the
[Creative Commons Attribution-ShareAlike 4.0 International][cc-by-sa-4.0].

## Requirements
* `cmake` version `3.10` or newer.
* Either
    * a compiler that supports `c++17` and `libboost` version `1.56.0` or newer
      for `boost::span` or
    * a compiler that supports `c++20` which provides the feature `std::span`.

## Building
To build the examples, run:
```sh
cmake -B build
cmake --build build
```

## Usage
To use the `sink.epitome.output` component, we can use the command:
```sh
babeltrace2 --plugin-path=build --component=sink.epitome.output /path/to/ctf/trace
```


[sample-sink-component]: https://babeltrace.org/docs/v2.1/libbabeltrace2/example-simple-sink-cmp-cls.html
[epl-2.0]: https://www.eclipse.org/legal/epl-2.0/
[cc-by-sa-4.0]: https://creativecommons.org/licenses/by-sa/4.0/
