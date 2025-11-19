// Reset exercise counter for each day
document.addEventListener('DOMContentLoaded', function() {
    // Get the current page filename
    var currentPage = window.location.pathname.split('/').pop();

    // Define offset for each day
    var offsets = {
        'robot-programming-3-2025.html': 0,  // Day 1: starts from 1 (no offset)
        'robot-programming-1-2025.html': -7, // Day 2: should start from 1 (offset -7)
        'robot-programming-2-2025.html': -15 // Day 3: should start from 1 (offset -15)
    };

    // Check if current page needs exercise number adjustment
    if (currentPage in offsets) {
        var offset = offsets[currentPage];

        // Find all exercise caption numbers
        var captionNumbers = document.querySelectorAll('.exercise .admonition-title .caption-number');

        captionNumbers.forEach(function(element) {
            // Extract current number
            var text = element.textContent;
            var match = text.match(/チェックポイント (\d+)/);

            if (match) {
                var currentNumber = parseInt(match[1]);
                var newNumber = currentNumber + offset;

                // Update the text with new number
                element.textContent = 'チェックポイント ' + newNumber + ' ';
            }
        });
    }
});
