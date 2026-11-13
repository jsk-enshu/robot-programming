// Number the checkpoints of each page from 1.
//
// sphinx-exercise numbers exercises across the whole book, so a page's first
// checkpoint gets the number after the last one of the pages before it. This
// renumbers them in page order instead of subtracting a fixed offset per page,
// which went wrong whenever a page was added, moved or given another checkpoint.
document.addEventListener('DOMContentLoaded', function() {
    var captionNumbers = document.querySelectorAll('.exercise .admonition-title .caption-number');
    var n = 0;
    captionNumbers.forEach(function(element) {
        if (/チェックポイント \d+/.test(element.textContent)) {
            n += 1;
            element.textContent = 'チェックポイント ' + n + ' ';
        }
    });
});
