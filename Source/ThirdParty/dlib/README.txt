The dlib code was taken from commit hash 96a75568be7701dedb94d84b6584af8dfa92c7db (Nov 18th 2020) of https://github.com/davisking/dlib

The following diff was applied to image_transforms/morphological_operations.h to correct a macro clash:

854c854
<         auto check = [&](long r, long c)
---
>         auto localCheck = [&](long r, long c) //@UE5 changed from just "check" to avoid a clash when used in UE
864,871c864,871
<         check(r-1,c-1);
<         check(r-1,c);
<         check(r-1,c+1);
<         check(r,c+1);
<         check(r+1,c+1);
<         check(r+1,c);
<         check(r+1,c-1);
<         check(r,c-1);
---
>         localCheck(r-1,c-1);
>         localCheck(r-1,c);
>         localCheck(r-1,c+1);
>         localCheck(r,c+1);
>         localCheck(r+1,c+1);
>         localCheck(r+1,c);
>         localCheck(r+1,c-1);
>         localCheck(r,c-1);

The following code has been added to dlib/Include/dlib/dnn/core.h
#if defined(_MSC_VER) && !defined(__clang__) instead of #ifdef _MSC_VER (line 23)
This is to fix the clang build in UE5