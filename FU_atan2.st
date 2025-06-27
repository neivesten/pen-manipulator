FUNCTION FU_atan2 : REAL
VAR_INPUT
    y : REAL;
    x : REAL;
END_VAR
VAR
    result : REAL;
END_VAR

IF x > 0 THEN
    result := ATAN(y / x);
ELSIF x < 0 AND y >= 0 THEN
    result := ATAN(y / x) + 3.1415926;
ELSIF x < 0 AND y < 0 THEN
    result := ATAN(y / x) - 3.1415926;
ELSIF x = 0 AND y > 0 THEN
    result := 3.1415926 / 2;
ELSIF x = 0 AND y < 0 THEN
    result := -3.1415926 / 2;
ELSE
    result := 0.0;
END_IF;
FU_atan2 := result;