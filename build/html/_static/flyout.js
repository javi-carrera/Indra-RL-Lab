$(document).ready(function() {
    $('.rst-current-version').on('click', function(e) {
        e.preventDefault();
        $('.rst-other-versions').toggle(); // Toggle visibility of the flyout
    });

    // Optional: close the menu if you click outside of it
    $(document).on('click', function(e) {
        if (!$(e.target).closest('.rst-versions').length) {
            $('.rst-other-versions').hide(); // Close the menu when clicking outside
        }
    });
});
