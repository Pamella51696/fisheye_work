package surround.calibration;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Minimal JSON parser for rig config (objects, arrays, numbers, strings). */
public final class JsonUtil {

    private JsonUtil() {
    }

    static Object parse(String json) {
        return new Parser(json).parseValue();
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> asObject(Object o) {
        return (Map<String, Object>) o;
    }

    @SuppressWarnings("unchecked")
    static List<Object> asArray(Object o) {
        return (List<Object>) o;
    }

    static double asDouble(Object o) {
        if (o instanceof Number) {
            return ((Number) o).doubleValue();
        }
        return Double.parseDouble(o.toString());
    }

    static String asString(Object o) {
        return o.toString();
    }

    static double asDouble(Map<String, Object> map, String key, double fallback) {
        Object o = map.get(key);
        return o == null ? fallback : asDouble(o);
    }

    private static final class Parser {
        private final String s;
        private int i;

        Parser(String s) {
            this.s = s.trim();
        }

        Object parseValue() {
            skipWs();
            char c = s.charAt(i);
            if (c == '{') {
                return parseObject();
            }
            if (c == '[') {
                return parseArray();
            }
            if (c == '"') {
                return parseString();
            }
            return parseNumber();
        }

        Map<String, Object> parseObject() {
            Map<String, Object> map = new HashMap<>();
            i++; // {
            skipWs();
            if (s.charAt(i) == '}') {
                i++;
                return map;
            }
            while (true) {
                skipWs();
                String key = parseString();
                skipWs();
                expect(':');
                Object val = parseValue();
                map.put(key, val);
                skipWs();
                if (s.charAt(i) == '}') {
                    i++;
                    break;
                }
                expect(',');
            }
            return map;
        }

        List<Object> parseArray() {
            List<Object> list = new ArrayList<>();
            i++; // [
            skipWs();
            if (s.charAt(i) == ']') {
                i++;
                return list;
            }
            while (true) {
                list.add(parseValue());
                skipWs();
                if (s.charAt(i) == ']') {
                    i++;
                    break;
                }
                expect(',');
            }
            return list;
        }

        String parseString() {
            i++; // opening quote
            StringBuilder sb = new StringBuilder();
            while (s.charAt(i) != '"') {
                char c = s.charAt(i++);
                if (c == '\\') {
                    sb.append(s.charAt(i++));
                } else {
                    sb.append(c);
                }
            }
            i++; // closing quote
            return sb.toString();
        }

        Number parseNumber() {
            Matcher m = Pattern.compile("-?[0-9]+(?:\\.[0-9]+)?(?:[eE][-+]?[0-9]+)?")
                    .matcher(s.substring(i));
            if (!m.find()) {
                throw new IllegalArgumentException("expected number at " + i);
            }
            String num = m.group();
            i += num.length();
            if (num.contains(".") || num.contains("e") || num.contains("E")) {
                return Double.parseDouble(num);
            }
            return Long.parseLong(num);
        }

        void skipWs() {
            while (i < s.length() && Character.isWhitespace(s.charAt(i))) {
                i++;
            }
        }

        void expect(char c) {
            skipWs();
            if (s.charAt(i) != c) {
                throw new IllegalArgumentException("expected '" + c + "' at " + i);
            }
            i++;
        }
    }
}
