$(document).ready(function() {
  //Implement the showing and hiding of the sidebar when the user clicks on #sidebar-button here:
  $("#sidebar-button").click(function(){
    if ($(".sidebar-container").hasClass("sidebar-active")){
      $("body").removeClass("no-scroll");
      $("#sidebar-button").removeClass("button-active");
      $(".sidebar-container").removeClass("sidebar-active");
      $(".page-wrapper").removeClass("wrapper-active");
    } else {
      $("#sidebar-button").addClass("button-active");
      $(".sidebar-container").addClass("sidebar-active");
      $(".page-wrapper").addClass("wrapper-active");
      setTimeout(function () {
        $('body').addClass('no-scroll');
      }, 300);
    }
  });

$('#abstract-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#abstract-section").offset().top}, 800);
});
$('#title-button').click(function(){
  ($('html,body')).animate({scrollTop: 0}, 800);
});
$('#tech-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#tech-section").offset().top}, 800);
});
$('#point-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#point-section").offset().top}, 800);
});
$('#oct-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#oct-section").offset().top}, 800);
});
$('#BSSRDF-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#BSSRDF-section").offset().top}, 800);
});
$('#multiple-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#multiple-section").offset().top}, 800);
});
$('#single-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#single-section").offset().top}, 800);
});
$('#diff-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#diff-section").offset().top}, 800);
});
$('#lesson-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#lesson-section").offset().top}, 800);
});
$('#result-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#result-section").offset().top}, 800);
});
$('#ref-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#ref-section").offset().top}, 800);
});
$('#resp-button').click(function(){
  ($('html,body')).animate({scrollTop: $("#resp-section").offset().top}, 800);
});




  //Implement the hiding of the sidebar when the user clicks on the page wrapper here:
  $(".page-wrapper").click(function(){
    if ($(".sidebar-container").hasClass("sidebar-active")){
      $("body").removeClass("no-scroll");
      $("#sidebar-button").removeClass("button-active");
      $(".sidebar-container").removeClass("sidebar-active");
      $(".page-wrapper").removeClass("wrapper-active");
    }
  });

});