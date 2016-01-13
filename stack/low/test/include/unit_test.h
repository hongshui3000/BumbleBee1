#ifndef UNIT_TEST_H_
#define UNIT_TEST_H_

#ifdef UNIT_TEST
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#else /* ! UNIT_TEST*/
/*
 * Disable all assert macros in <cmocka.h>
 */
#define assert_true(c)
#define assert_false(c)
#define assert_return_code(rc, error)
#define assert_non_null(c)
#define assert_null(c)
#define assert_ptr_equal(a, b)
#define assert_ptr_not_equal(a, b)
#define assert_int_equal(a, b)
#define assert_int_not_equal(a, b)
#define assert_string_equal(a, b)
#define assert_string_not_equal(a, b)
#define assert_memory_equal(a, b, size)
#define assert_memory_not_equal(a, b, size)
#define assert_in_range(value, minimum, maximum)
#define assert_not_in_range(value, minimum, maximum)
#define assert_in_set(value, values, number_of_values)
#define assert_not_in_set(value, values, number_of_values)

#endif /* UNIT_TEST */

#endif /* UNIT_TEST_H_ */
