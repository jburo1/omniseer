(function () {
  "use strict";

  var primaryToggleSelector =
    ".md-sidebar--primary .md-nav--primary > .md-nav__list > " +
    ".md-nav__item--nested > .md-nav__toggle";

  function syncExpandedState(toggle) {
    var nav = toggle.parentElement.querySelector(":scope > .md-nav");
    if (nav) {
      nav.setAttribute("aria-expanded", toggle.checked ? "true" : "false");
    }
  }

  function bindToggle(toggle) {
    if (toggle.dataset.omniseerCollapseBound) {
      return;
    }

    toggle.addEventListener("change", function () {
      syncExpandedState(toggle);
    });
    toggle.dataset.omniseerCollapseBound = "true";
  }

  function collapsePrimarySidebarGroups() {
    document.querySelectorAll(primaryToggleSelector).forEach(function (toggle) {
      toggle.checked = false;
      toggle.indeterminate = false;
      toggle.classList.remove("md-toggle--indeterminate");
      bindToggle(toggle);
      syncExpandedState(toggle);
    });
  }

  if (typeof document$ !== "undefined") {
    document$.subscribe(collapsePrimarySidebarGroups);
  } else {
    document.addEventListener("DOMContentLoaded", collapsePrimarySidebarGroups);
  }
})();
